#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace sensor_msgs::msg;
using namespace livox_ros_driver2::msg;
using namespace message_filters;

class SyncNode : public rclcpp::Node {
public:
    SyncNode() : Node("sync_node"), sync(MySyncPolicy(50), image_sub, lidar_sub) {
        // Get parameters from the parameter server
        this->declare_parameter("image_topic", "/camera/camera/color/image_raw");
        this->declare_parameter("lidar_topic", "/livox/lidar");
        this->declare_parameter("synced_image_topic", "/synced_image");
        this->declare_parameter("synced_lidar_topic", "/synced_lidar");
        this->declare_parameter("publish_rate", 10.0);
        this->declare_parameter("max_time_diff", 0.02); // 20ms max time difference

        std::string image_topic = this->get_parameter("image_topic").as_string();
        std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
        std::string synced_image_topic = this->get_parameter("synced_image_topic").as_string();
        std::string synced_lidar_topic = this->get_parameter("synced_lidar_topic").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        double max_time_diff = this->get_parameter("max_time_diff").as_double();

        RCLCPP_INFO(this->get_logger(), "Sync node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to image topic: %s", image_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribing to lidar topic: %s", lidar_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing synced image to: %s", synced_image_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing synced lidar to: %s", synced_lidar_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Max time difference: %.1f ms", max_time_diff * 1000);

        image_sub.subscribe(this, image_topic, rmw_qos_profile_sensor_data);
        lidar_sub.subscribe(this, lidar_topic, rmw_qos_profile_sensor_data);

        image_pub = this->create_publisher<Image>(synced_image_topic, 10);
        lidar_pub = this->create_publisher<CustomMsg>(synced_lidar_topic, 10);

        // Configure sync policy with tighter time tolerance
        sync.setMaxIntervalDuration(rclcpp::Duration::from_nanoseconds(max_time_diff * 1e9));
        sync.registerCallback(std::bind(&SyncNode::callback, this, std::placeholders::_1, std::placeholders::_2));

        // Set up a timer to publish synchronized messages at the specified rate
        auto period = std::chrono::duration<double>(1.0 / publish_rate);
        timer = this->create_wall_timer(period, std::bind(&SyncNode::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Sync node setup complete, publish rate: %.1f Hz", publish_rate);
    }

private:
    Subscriber<Image> image_sub;
    Subscriber<CustomMsg> lidar_sub;
    typedef sync_policies::ApproximateTime<Image, CustomMsg> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync;
    rclcpp::Publisher<Image>::SharedPtr image_pub;
    rclcpp::Publisher<CustomMsg>::SharedPtr lidar_pub;
    rclcpp::TimerBase::SharedPtr timer;

    Image last_image_msg;
    CustomMsg last_lidar_msg;
    bool new_image_received = false;
    bool new_lidar_received = false;

    void callback(const Image::ConstSharedPtr& img_msg, const CustomMsg::ConstSharedPtr& lidar_msg) {
        // Calculate time difference
        double img_time = img_msg->header.stamp.sec + img_msg->header.stamp.nanosec * 1e-9;
        double lidar_time = lidar_msg->header.stamp.sec + lidar_msg->header.stamp.nanosec * 1e-9;
        double time_diff = std::abs(img_time - lidar_time) * 1000; // in milliseconds
        
        // Store the latest synchronized messages
        last_image_msg = *img_msg;
        last_lidar_msg = *lidar_msg;
        new_image_received = true;
        new_lidar_received = true;
        
        RCLCPP_INFO(this->get_logger(), "Synchronized messages received - Image: %d.%09d, Lidar: %d.%09d, Time diff: %.1f ms", 
                    img_msg->header.stamp.sec, img_msg->header.stamp.nanosec,
                    lidar_msg->header.stamp.sec, lidar_msg->header.stamp.nanosec,
                    time_diff);
    }

    void timerCallback() {
        if (new_image_received && new_lidar_received) {
            image_pub->publish(last_image_msg);
            lidar_pub->publish(last_lidar_msg);
            RCLCPP_INFO(this->get_logger(), "Published synchronized messages");
            new_image_received = false;
            new_lidar_received = false;
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Waiting for sync - Image: %s, Lidar: %s", 
                                 new_image_received ? "OK" : "Missing",
                                 new_lidar_received ? "OK" : "Missing");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SyncNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 