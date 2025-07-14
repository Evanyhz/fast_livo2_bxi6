#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <unordered_map>

struct GlobalGridCell {
    float max_elevation = std::numeric_limits<float>::lowest();
    float avg_elevation = 0.0f;
    int point_count = 0;
    rclcpp::Time last_update;
    bool has_data = false;
    
    void addPoint(float elevation, const rclcpp::Time& time) {
        if (!has_data) {
            max_elevation = elevation;
            avg_elevation = elevation;
            point_count = 1;
            has_data = true;
        } else {
            max_elevation = std::max(max_elevation, elevation);
            avg_elevation = (avg_elevation * point_count + elevation) / (point_count + 1);
            point_count++;
        }
        last_update = time;
    }
    
    bool isValid() const {
        return has_data && point_count > 0;
    }
};

// 全局网格键的哈希函数
struct GridKeyHash {
    std::size_t operator()(const std::pair<int, int>& k) const {
        return std::hash<int>()(k.first) ^ (std::hash<int>()(k.second) << 1);
    }
};

class ElevationMapGenerator : public rclcpp::Node
{
public:
    ElevationMapGenerator() : Node("elevation_map_generator")
    {
        // 参数配置
        this->declare_parameter("local_grid_size", 0.1);
        this->declare_parameter("local_map_width", 1.0);
        this->declare_parameter("local_map_height", 1.6);
        this->declare_parameter("global_grid_size", 0.2);
        this->declare_parameter("global_map_width", 100.0);
        this->declare_parameter("global_map_height", 100.0);
        
        // 获取地图尺寸参数
        global_map_width_ = this->get_parameter("global_map_width").as_double();
        global_map_height_ = this->get_parameter("global_map_height").as_double();
        
        // 设置原点为几何中心
        this->declare_parameter("global_map_origin_x", -global_map_width_ / 2.0);
        this->declare_parameter("global_map_origin_y", -global_map_height_ / 2.0);
        this->declare_parameter("data_timeout_sec", 300.0);
        this->declare_parameter("min_points_per_cell", 3);
        this->declare_parameter("publish_global_map", true);
        this->declare_parameter("use_max_elevation", true);
        
        local_grid_size_ = this->get_parameter("local_grid_size").as_double();
        local_map_width_ = this->get_parameter("local_map_width").as_double();
        local_map_height_ = this->get_parameter("local_map_height").as_double();
        global_grid_size_ = this->get_parameter("global_grid_size").as_double();
        global_map_origin_x_ = this->get_parameter("global_map_origin_x").as_double();
        global_map_origin_y_ = this->get_parameter("global_map_origin_y").as_double();
        data_timeout_sec_ = this->get_parameter("data_timeout_sec").as_double();
        min_points_per_cell_ = this->get_parameter("min_points_per_cell").as_int();
        publish_global_map_ = this->get_parameter("publish_global_map").as_bool();
        use_max_elevation_ = this->get_parameter("use_max_elevation").as_bool();
        
        // 计算局部网格数量
        local_grid_width_ = static_cast<int>(local_map_width_ / local_grid_size_);
        local_grid_height_ = static_cast<int>(local_map_height_ / local_grid_size_);
        
        // 计算全局网格数量
        global_grid_width_ = static_cast<int>(global_map_width_ / global_grid_size_);
        global_grid_height_ = static_cast<int>(global_map_height_ / global_grid_size_);
        
        // 初始化局部高程图
        local_elevation_map_.resize(local_grid_width_ * local_grid_height_, std::numeric_limits<float>::lowest());
        
        // 订阅话题
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 10,
            std::bind(&ElevationMapGenerator::cloudCallback, this, std::placeholders::_1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/aft_mapped_to_init", 10,
            std::bind(&ElevationMapGenerator::odomCallback, this, std::placeholders::_1));
        
        // 发布话题
        local_elevation_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/elevation_map_local", 10);
            
        if (publish_global_map_) {
            global_elevation_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                "/elevation_map_global", 10);
        }
            
        // 定时器
        local_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ElevationMapGenerator::publishLocalElevationMap, this));
            
        if (publish_global_map_) {
            global_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000),
                std::bind(&ElevationMapGenerator::publishGlobalElevationMap, this));
        }
        
        // 清理定时器
        cleanup_timer_ = this->create_wall_timer(
            std::chrono::seconds(30),
            std::bind(&ElevationMapGenerator::cleanupOldData, this));
            
        RCLCPP_INFO(this->get_logger(), "ElevationMapGenerator initialized");
        RCLCPP_INFO(this->get_logger(), "Local: %.2f m grid, %.1f x %.1f m area", 
                   local_grid_size_, local_map_width_, local_map_height_);
        RCLCPP_INFO(this->get_logger(), "Global: %.2f m grid, %.1f x %.1f m map, centered at origin (%.1f, %.1f)", 
                   global_grid_size_, global_map_width_, global_map_height_, 
                   global_map_origin_x_, global_map_origin_y_);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = *msg;
        has_odom_ = true;
    }
    
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!has_odom_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                "No odometry data received yet");
            return;
        }
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        try {
            pcl::fromROSMsg(*msg, *cloud);
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Failed to convert point cloud: %s", e.what());
            return;
        }
        
        // 获取当前时间
        rclcpp::Time current_time = this->get_clock()->now();
        
        // 获取当前机器人位置
        double robot_x = current_pose_.pose.pose.position.x;
        double robot_y = current_pose_.pose.pose.position.y;
        
        // 获取机器人朝向
        tf2::Quaternion q(
            current_pose_.pose.pose.orientation.x,
            current_pose_.pose.pose.orientation.y,
            current_pose_.pose.pose.orientation.z,
            current_pose_.pose.pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        // 重置局部高程图
        std::fill(local_elevation_map_.begin(), local_elevation_map_.end(), std::numeric_limits<float>::lowest());
        
        // 处理每个点
        for (const auto& point : cloud->points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                continue;
                
            // 更新全局高程图（使用世界坐标）
            updateGlobalMap(point.x, point.y, point.z, current_time);
            
            // 更新局部高程图（使用机器人坐标系）
            updateLocalMap(point.x, point.y, point.z, robot_x, robot_y, yaw);
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Processed %zu points, global map size: %zu", 
                    cloud->size(), global_elevation_map_.size());
    }
    
    void updateGlobalMap(double world_x, double world_y, double z, const rclcpp::Time& time)
    {
        // 检查点是否在全局地图范围内
        if (world_x < global_map_origin_x_ || 
            world_x >= global_map_origin_x_ + global_map_width_ ||
            world_y < global_map_origin_y_ || 
            world_y >= global_map_origin_y_ + global_map_height_) {
            return;
        }
        
        // 计算全局网格坐标
        int grid_x = static_cast<int>((world_x - global_map_origin_x_) / global_grid_size_);
        int grid_y = static_cast<int>((world_y - global_map_origin_y_) / global_grid_size_);
        
        std::pair<int, int> key = {grid_x, grid_y};
        
        // 更新或创建网格单元
        auto& cell = global_elevation_map_[key];
        cell.addPoint(z, time);
    }
    
    void updateLocalMap(double x, double y, double z, double robot_x, double robot_y, double yaw)
    {
        // 将点转换到机器人坐标系
        double dx = x - robot_x;
        double dy = y - robot_y;
        
        // 旋转到机器人朝向
        double local_x = dx * cos(-yaw) - dy * sin(-yaw);
        double local_y = dx * sin(-yaw) + dy * cos(-yaw);
        
        // 检查点是否在局部地图范围内
        if (local_x < -local_map_height_/2 || local_x > local_map_height_/2 || 
            local_y < -local_map_width_/2 || local_y > local_map_width_/2)
            return;
            
        // 转换到局部网格坐标
        int grid_x = static_cast<int>((local_x + local_map_height_/2) / local_grid_size_);
        int grid_y = static_cast<int>((local_y + local_map_width_/2) / local_grid_size_);
        
        // 边界检查
        if (grid_x < 0 || grid_x >= local_grid_height_ || grid_y < 0 || grid_y >= local_grid_width_)
            return;
            
        int index = grid_x * local_grid_width_ + grid_y;
        
        // 更新局部最大高程值
        if (local_elevation_map_[index] == std::numeric_limits<float>::lowest() || z > local_elevation_map_[index])
        {
            local_elevation_map_[index] = z;
        }
    }
    
    void publishLocalElevationMap()
    {
        if (!has_odom_) return;
        
        nav_msgs::msg::OccupancyGrid grid_msg;
        grid_msg.header.stamp = this->get_clock()->now();
        grid_msg.header.frame_id = "map";
        
        // 设置地图元信息
        grid_msg.info.resolution = local_grid_size_;
        grid_msg.info.width = local_grid_width_;
        grid_msg.info.height = local_grid_height_;
        
        // 设置地图原点（跟随机器人）
        grid_msg.info.origin.position.x = current_pose_.pose.pose.position.x - local_map_height_/2;
        grid_msg.info.origin.position.y = current_pose_.pose.pose.position.y - local_map_width_/2;
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation = current_pose_.pose.pose.orientation;
        
        // 填充地图数据
        normalizeAndFillGrid(local_elevation_map_, grid_msg);
        
        local_elevation_pub_->publish(grid_msg);
    }
    
    void publishGlobalElevationMap()
    {
        if (global_elevation_map_.empty()) return;
        
        nav_msgs::msg::OccupancyGrid grid_msg;
        grid_msg.header.stamp = this->get_clock()->now();
        grid_msg.header.frame_id = "map";
        
        // 设置全局地图元信息（固定坐标系）
        grid_msg.info.resolution = global_grid_size_;
        grid_msg.info.width = global_grid_width_;
        grid_msg.info.height = global_grid_height_;
        
        // 设置固定的地图原点
        grid_msg.info.origin.position.x = global_map_origin_x_;
        grid_msg.info.origin.position.y = global_map_origin_y_;
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;
        
        // 构建全局地图数据
        std::vector<float> global_map_data(global_grid_width_ * global_grid_height_, 
                                          std::numeric_limits<float>::lowest());
        
        // 将哈希表数据转换到网格数组
        for (const auto& [key, cell] : global_elevation_map_) {
            if (cell.isValid() && cell.point_count >= min_points_per_cell_) {
                int grid_x = key.first;
                int grid_y = key.second;
                
                // 检查索引范围
                if (grid_x >= 0 && grid_x < global_grid_width_ && 
                    grid_y >= 0 && grid_y < global_grid_height_) {
                    int index = grid_y * global_grid_width_ + grid_x;
                    global_map_data[index] = use_max_elevation_ ? 
                                           cell.max_elevation : cell.avg_elevation;
                }
            }
        }
        
        // 填充地图数据
        normalizeAndFillGrid(global_map_data, grid_msg);
        
        global_elevation_pub_->publish(grid_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Published global map: %dx%d, %zu known cells", 
                    grid_msg.info.width, grid_msg.info.height, global_elevation_map_.size());
    }
    
    void normalizeAndFillGrid(const std::vector<float>& elevation_data, nav_msgs::msg::OccupancyGrid& grid_msg)
    {
        grid_msg.data.resize(elevation_data.size());
        
        // 找到高程范围
        float min_elevation = std::numeric_limits<float>::max();
        float max_elevation = std::numeric_limits<float>::lowest();
        
        for (float elevation : elevation_data) {
            if (elevation != std::numeric_limits<float>::lowest()) {
                min_elevation = std::min(min_elevation, elevation);
                max_elevation = std::max(max_elevation, elevation);
            }
        }
        
        // 归一化到0-100范围
        for (size_t i = 0; i < elevation_data.size(); ++i) {
            if (elevation_data[i] == std::numeric_limits<float>::lowest()) {
                grid_msg.data[i] = -1;  // 未知区域
            } else {
                if (max_elevation > min_elevation) {
                    float normalized = (elevation_data[i] - min_elevation) / (max_elevation - min_elevation);
                    grid_msg.data[i] = static_cast<int8_t>(normalized * 100);
                } else {
                    grid_msg.data[i] = 50;  // 平坦区域
                }
            }
        }
    }
    
    void cleanupOldData()
    {
        if (data_timeout_sec_ <= 0) return;
        
        rclcpp::Time current_time = this->get_clock()->now();
        rclcpp::Duration timeout = rclcpp::Duration::from_seconds(data_timeout_sec_);
        
        auto it = global_elevation_map_.begin();
        while (it != global_elevation_map_.end()) {
            if ((current_time - it->second.last_update) > timeout) {
                it = global_elevation_map_.erase(it);
            } else {
                ++it;
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Cleanup: global map now has %zu cells", 
                    global_elevation_map_.size());
    }

private:
    // 订阅者和发布者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_elevation_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_elevation_pub_;
    rclcpp::TimerBase::SharedPtr local_timer_;
    rclcpp::TimerBase::SharedPtr global_timer_;
    rclcpp::TimerBase::SharedPtr cleanup_timer_;
    
    // 参数
    double local_grid_size_;      // 局部网格大小
    double local_map_width_;      // 局部地图宽度
    double local_map_height_;     // 局部地图高度
    double global_grid_size_;     // 全局网格大小
    double global_map_origin_x_;  // 全局地图原点X
    double global_map_origin_y_;  // 全局地图原点Y
    double global_map_width_;     // 全局地图宽度
    double global_map_height_;    // 全局地图高度
    double data_timeout_sec_;     // 数据超时时间
    int min_points_per_cell_;     // 每个网格最少点数
    bool publish_global_map_;     // 是否发布全局地图
    bool use_max_elevation_;      // 使用最大高程还是平均高程
    
    int local_grid_width_;        // 局部网格宽度数量
    int local_grid_height_;       // 局部网格高度数量
    int global_grid_width_;       // 全局网格宽度数量
    int global_grid_height_;      // 全局网格高度数量
    
    // 数据存储
    std::vector<float> local_elevation_map_;                                         // 局部高程数据
    std::unordered_map<std::pair<int, int>, GlobalGridCell, GridKeyHash> global_elevation_map_;  // 全局高程数据
    nav_msgs::msg::Odometry current_pose_;                                          // 当前位姿
    bool has_odom_ = false;                                                         // 是否接收到里程计数据
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ElevationMapGenerator>());
    rclcpp::shutdown();
    return 0;
} 