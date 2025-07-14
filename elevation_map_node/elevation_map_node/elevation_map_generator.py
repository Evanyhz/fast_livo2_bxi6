#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import numpy as np
import math
from typing import Dict, Tuple, Optional
from collections import defaultdict
import time

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header

# 用于点云处理
import struct


class GlobalGridCell:
    """全局网格单元类，对应C++中的GlobalGridCell结构体"""
    
    def __init__(self):
        self.max_elevation = float('-inf')
        self.avg_elevation = 0.0
        self.point_count = 0
        self.last_update = None
        self.has_data = False
    
    def add_point(self, elevation: float, time: Time):
        """添加点到网格单元"""
        if not self.has_data:
            self.max_elevation = elevation
            self.avg_elevation = elevation
            self.point_count = 1
            self.has_data = True
        else:
            self.max_elevation = max(self.max_elevation, elevation)
            self.avg_elevation = (self.avg_elevation * self.point_count + elevation) / (self.point_count + 1)
            self.point_count += 1
        self.last_update = time
    
    def is_valid(self) -> bool:
        """检查网格单元是否有效"""
        return self.has_data and self.point_count > 0


class ElevationMapGenerator(Node):
    """高程图生成器节点"""
    
    def __init__(self):
        super().__init__('elevation_map_generator')
        
        # 声明参数
        self.declare_parameter('local_grid_size', 0.1)
        self.declare_parameter('local_map_width', 1.0)
        self.declare_parameter('local_map_height', 1.6)
        self.declare_parameter('global_grid_size', 0.2)
        self.declare_parameter('global_map_width', 100.0)
        self.declare_parameter('global_map_height', 100.0)
        
        # 获取地图尺寸参数
        self.global_map_width = self.get_parameter('global_map_width').get_parameter_value().double_value
        self.global_map_height = self.get_parameter('global_map_height').get_parameter_value().double_value
        
        # 设置原点为几何中心
        self.declare_parameter('global_map_origin_x', -self.global_map_width / 2.0)
        self.declare_parameter('global_map_origin_y', -self.global_map_height / 2.0)
        self.declare_parameter('data_timeout_sec', 300.0)
        self.declare_parameter('min_points_per_cell', 3)
        self.declare_parameter('publish_global_map', True)
        self.declare_parameter('use_max_elevation', True)
        
        # 获取参数值
        self.local_grid_size = self.get_parameter('local_grid_size').get_parameter_value().double_value
        self.local_map_width = self.get_parameter('local_map_width').get_parameter_value().double_value
        self.local_map_height = self.get_parameter('local_map_height').get_parameter_value().double_value
        self.global_grid_size = self.get_parameter('global_grid_size').get_parameter_value().double_value
        self.global_map_origin_x = self.get_parameter('global_map_origin_x').get_parameter_value().double_value
        self.global_map_origin_y = self.get_parameter('global_map_origin_y').get_parameter_value().double_value
        self.data_timeout_sec = self.get_parameter('data_timeout_sec').get_parameter_value().double_value
        self.min_points_per_cell = self.get_parameter('min_points_per_cell').get_parameter_value().integer_value
        self.publish_global_map = self.get_parameter('publish_global_map').get_parameter_value().bool_value
        self.use_max_elevation = self.get_parameter('use_max_elevation').get_parameter_value().bool_value
        
        # 计算网格数量
        self.local_grid_width = int(self.local_map_width / self.local_grid_size)
        self.local_grid_height = int(self.local_map_height / self.local_grid_size)
        self.global_grid_width = int(self.global_map_width / self.global_grid_size)
        self.global_grid_height = int(self.global_map_height / self.global_grid_size)
        
        # 初始化数据存储
        self.local_elevation_map = np.full(
            (self.local_grid_height, self.local_grid_width), 
            float('-inf'), 
            dtype=np.float32
        )
        self.global_elevation_map: Dict[Tuple[int, int], GlobalGridCell] = {}
        self.current_pose: Optional[Odometry] = None
        self.last_local_map_pose: Optional[Tuple[float, float, float]] = None  # (x, y, yaw)
        self.has_odom = False
        
        # 局部地图更新阈值
        self.local_map_update_distance = 0.2  # 机器人移动超过20cm时更新局部地图
        self.local_map_update_angle = 0.3     # 机器人旋转超过0.3弧度时更新局部地图
        
        # 创建订阅者
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/cloud_registered',
            self.cloud_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/aft_mapped_to_init',
            self.odom_callback,
            10
        )
        
        # 创建发布者
        self.local_elevation_pub = self.create_publisher(
            OccupancyGrid,
            '/elevation_map_local',
            10
        )
        
        if self.publish_global_map:
            self.global_elevation_pub = self.create_publisher(
                OccupancyGrid,
                '/elevation_map_global',
                10
            )
        
        # 创建定时器
        self.local_timer = self.create_timer(0.1, self.publish_local_elevation_map)
        
        if self.publish_global_map:
            self.global_timer = self.create_timer(1.0, self.publish_global_elevation_map)
        
        # 清理定时器
        self.cleanup_timer = self.create_timer(30.0, self.cleanup_old_data)
        
        # 日志信息
        self.get_logger().info('ElevationMapGenerator initialized')
        self.get_logger().info(f'Local: {self.local_grid_size:.2f} m grid, {self.local_map_width:.1f} x {self.local_map_height:.1f} m area')
        self.get_logger().info(f'Global: {self.global_grid_size:.2f} m grid, {self.global_map_width:.1f} x {self.global_map_height:.1f} m map, centered at origin ({self.global_map_origin_x:.1f}, {self.global_map_origin_y:.1f})')
    
    def odom_callback(self, msg: Odometry):
        """里程计回调函数"""
        self.current_pose = msg
        self.has_odom = True
    
    def cloud_callback(self, msg: PointCloud2):
        """点云回调函数"""
        if not self.has_odom:
            self.get_logger().warn('No odometry data received yet', throttle_duration_sec=2.0)
            return
        
        try:
            # 解析点云数据
            points = self.parse_pointcloud2(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to convert point cloud: {e}', throttle_duration_sec=5.0)
            return
        
        # 获取当前时间
        current_time = self.get_clock().now()
        
        # 获取当前机器人位置
        robot_x = self.current_pose.pose.pose.position.x
        robot_y = self.current_pose.pose.pose.position.y
        
        # 获取机器人朝向
        orientation = self.current_pose.pose.pose.orientation
        yaw = self.quaternion_to_yaw(orientation)
        
        # 处理每个点
        valid_points = 0
        local_points_added = 0
        for point in points:
            x, y, z = point
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            
            # 更新全局高程图（使用世界坐标）
            self.update_global_map(x, y, z, current_time)
            
            # 更新局部高程图（使用机器人坐标系）
            if self.update_local_map(x, y, z, robot_x, robot_y, yaw):
                local_points_added += 1
            valid_points += 1
        
        self.get_logger().debug(f'Processed {valid_points} points, {local_points_added} added to local map, global map size: {len(self.global_elevation_map)}')
    
    def parse_pointcloud2(self, msg: PointCloud2):
        """解析PointCloud2消息"""
        points = []
        
        # 获取字段信息
        fields = {field.name: field for field in msg.fields}
        
        # 检查是否有x, y, z字段
        if 'x' not in fields or 'y' not in fields or 'z' not in fields:
            raise ValueError("PointCloud2 message missing x, y, or z fields")
        
        # 获取偏移量
        x_offset = fields['x'].offset
        y_offset = fields['y'].offset
        z_offset = fields['z'].offset
        
        point_step = msg.point_step
        data = msg.data
        
        # 解析每个点
        for i in range(0, len(data), point_step):
            if i + point_step > len(data):
                break
            
            # 提取x, y, z坐标
            x = struct.unpack('f', data[i + x_offset:i + x_offset + 4])[0]
            y = struct.unpack('f', data[i + y_offset:i + y_offset + 4])[0]
            z = struct.unpack('f', data[i + z_offset:i + z_offset + 4])[0]
            
            points.append((x, y, z))
        
        return points
    
    def quaternion_to_yaw(self, quat: Quaternion) -> float:
        """四元数转换为yaw角"""
        # 计算yaw角
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def update_global_map(self, world_x: float, world_y: float, z: float, time: Time):
        """更新全局高程图"""
        # 检查点是否在全局地图范围内
        if (world_x < self.global_map_origin_x or 
            world_x >= self.global_map_origin_x + self.global_map_width or
            world_y < self.global_map_origin_y or 
            world_y >= self.global_map_origin_y + self.global_map_height):
            return
        
        # 计算全局网格坐标
        grid_x = int((world_x - self.global_map_origin_x) / self.global_grid_size)
        grid_y = int((world_y - self.global_map_origin_y) / self.global_grid_size)
        
        key = (grid_x, grid_y)
        
        # 更新或创建网格单元
        if key not in self.global_elevation_map:
            self.global_elevation_map[key] = GlobalGridCell()
        
        self.global_elevation_map[key].add_point(z, time)
    
    def update_local_map(self, x: float, y: float, z: float, 
                        robot_x: float, robot_y: float, yaw: float) -> bool:
        """更新局部高程图，返回是否成功更新"""
        # 将点转换到机器人坐标系
        dx = x - robot_x
        dy = y - robot_y
        
        # 旋转到机器人朝向
        local_x = dx * math.cos(-yaw) - dy * math.sin(-yaw)
        local_y = dx * math.sin(-yaw) + dy * math.cos(-yaw)
        
        # 检查点是否在局部地图范围内
        if (local_x < -self.local_map_height/2 or local_x > self.local_map_height/2 or 
            local_y < -self.local_map_width/2 or local_y > self.local_map_width/2):
            return False
        
        # 转换到局部网格坐标
        grid_x = int((local_x + self.local_map_height/2) / self.local_grid_size)
        grid_y = int((local_y + self.local_map_width/2) / self.local_grid_size)
        
        # 边界检查
        if (grid_x < 0 or grid_x >= self.local_grid_height or 
            grid_y < 0 or grid_y >= self.local_grid_width):
            return False
        
        # 更新局部最大高程值
        current_value = self.local_elevation_map[grid_x, grid_y]
        if current_value == float('-inf') or z > current_value:
            self.local_elevation_map[grid_x, grid_y] = z
            return True
        
        # 即使没有更新高程值，点仍然在范围内
        return True
    
    def publish_local_elevation_map(self):
        """发布局部高程图"""
        if not self.has_odom:
            return
        
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'
        
        # 设置地图元信息
        grid_msg.info.resolution = self.local_grid_size
        grid_msg.info.width = self.local_grid_width
        grid_msg.info.height = self.local_grid_height
        
        # 设置地图原点（跟随机器人）
        grid_msg.info.origin.position.x = self.current_pose.pose.pose.position.x - self.local_map_height/2
        grid_msg.info.origin.position.y = self.current_pose.pose.pose.position.y - self.local_map_width/2
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation = self.current_pose.pose.pose.orientation
        
        # 填充地图数据
        self.normalize_and_fill_grid(self.local_elevation_map.flatten(), grid_msg)
        
        self.local_elevation_pub.publish(grid_msg)
    
    def publish_global_elevation_map(self):
        """发布全局高程图"""
        if not self.global_elevation_map:
            return
        
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'
        
        # 设置全局地图元信息（固定坐标系）
        grid_msg.info.resolution = self.global_grid_size
        grid_msg.info.width = self.global_grid_width
        grid_msg.info.height = self.global_grid_height
        
        # 设置固定的地图原点
        grid_msg.info.origin.position.x = self.global_map_origin_x
        grid_msg.info.origin.position.y = self.global_map_origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        # 构建全局地图数据
        global_map_data = np.full(
            self.global_grid_width * self.global_grid_height,
            float('-inf'),
            dtype=np.float32
        )
        
        # 将哈希表数据转换到网格数组
        for (grid_x, grid_y), cell in self.global_elevation_map.items():
            if cell.is_valid() and cell.point_count >= self.min_points_per_cell:
                # 检查索引范围
                if (0 <= grid_x < self.global_grid_width and 
                    0 <= grid_y < self.global_grid_height):
                    index = grid_y * self.global_grid_width + grid_x
                    global_map_data[index] = (cell.max_elevation if self.use_max_elevation 
                                            else cell.avg_elevation)
        
        # 填充地图数据
        self.normalize_and_fill_grid(global_map_data, grid_msg)
        
        self.global_elevation_pub.publish(grid_msg)
        
        self.get_logger().debug(f'Published global map: {grid_msg.info.width}x{grid_msg.info.height}, {len(self.global_elevation_map)} known cells')
    
    def normalize_and_fill_grid(self, elevation_data: np.ndarray, grid_msg: OccupancyGrid):
        """归一化并填充网格数据"""
        grid_msg.data = [0] * len(elevation_data)
        
        # 找到高程范围
        valid_elevations = elevation_data[elevation_data != float('-inf')]
        
        if len(valid_elevations) == 0:
            # 如果没有有效数据，全部设为未知
            grid_msg.data = [-1] * len(elevation_data)
            return
        
        min_elevation = np.min(valid_elevations)
        max_elevation = np.max(valid_elevations)
        
        # 归一化到0-100范围
        for i, elevation in enumerate(elevation_data):
            if elevation == float('-inf'):
                grid_msg.data[i] = -1  # 未知区域
            else:
                if max_elevation > min_elevation:
                    normalized = (elevation - min_elevation) / (max_elevation - min_elevation)
                    grid_msg.data[i] = int(normalized * 100)
                else:
                    grid_msg.data[i] = 50  # 平坦区域
    
    def cleanup_old_data(self):
        """清理过期数据"""
        if self.data_timeout_sec <= 0:
            return
        
        current_time = self.get_clock().now()
        timeout = Duration(seconds=self.data_timeout_sec)
        
        # 创建要删除的键列表
        keys_to_remove = []
        for key, cell in self.global_elevation_map.items():
            if current_time - cell.last_update > timeout:
                keys_to_remove.append(key)
        
        # 删除过期数据
        for key in keys_to_remove:
            del self.global_elevation_map[key]
        
        self.get_logger().debug(f'Cleanup: global map now has {len(self.global_elevation_map)} cells')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = ElevationMapGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 