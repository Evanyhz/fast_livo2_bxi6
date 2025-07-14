#!/usr/bin/env python3
"""
高程图生成器模块

该模块实现了一个ROS2节点，用于从激光雷达点云数据生成高精度的高程图。
主要功能包括：
1. 从点云数据生成局部高程图（跟随机器人移动的高分辨率地图）
2. 维护全局高程图（固定坐标系的历史累积地图）
3. 实时可视化地图数据
4. 发布ROS2格式的高程图消息

"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import numpy as np
import math
from typing import Dict, Tuple, Optional
from collections import defaultdict
import time

# ROS2消息类型
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header

# 用于解析点云二进制数据
import struct

# 可视化模块
from height_map_plot import HeightMapPlotter  # 绘图文件
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

class GlobalGridCell:
    """
    全局地图网格单元类
    
    该类用于存储全局地图中每个网格单元的点云历史数据，
    提供高程统计信息（最大值、平均值等）。
    采用增量式更新策略，适合长时间运行的SLAM系统。
    """

    def __init__(self):
        """初始化网格单元"""
        self.max_elevation = float('-inf')    # 该网格内观测到的最大高程值
        self.avg_elevation = 0.0              # 该网格内点云的平均高程值
        self.point_count = 0                  # 该网格内累积的点云数量
        self.last_update = None               # 最后一次更新的时间戳
        self.has_data = False                 # 标记该网格是否包含有效数据
    
    def add_point(self, new_elevation: float, time: Time):
        """
        向网格单元添加新的高程点
        
        Args:
            new_elevation (float): 新观测到的高程值
            time (Time): 观测时间戳
            
        采用增量式更新算法：
        - 首次添加点时直接设置为初始值
        - 后续点使用蒙特卡洛方法更新平均值，避免存储所有历史点
        """
        if not self.has_data:
            # 首次添加点：直接初始化
            self.max_elevation = new_elevation
            self.avg_elevation = new_elevation
            self.point_count = 1
            self.has_data = True
        else:
            # 增量更新：
            # 1. 更新最大值
            self.max_elevation = max(self.max_elevation, new_elevation)
            # 2. 使用蒙特卡洛方法更新平均值：新均值 = (旧均值*旧数量 + 新值) / 新数量
            self.avg_elevation = (self.avg_elevation * self.point_count + new_elevation) / (self.point_count + 1)
            # 3. 更新点数量
            self.point_count += 1
        # 更新时间戳
        self.last_update = time
    
    def is_valid(self) -> bool:
        """
        检查网格单元是否包含有效数据
        
        Returns:
            bool: True表示网格包含有效数据
        """
        return self.has_data and self.point_count > 0

class ElevationMapGenerator(Node):
    """
    高程图生成器主节点类
    
    该类实现了一个完整的高程图生成系统，包括：
    1. 点云数据接收和解析
    2. 坐标系转换（世界坐标系 <-> 机器人坐标系 <-> 网格坐标系）
    3. 双地图策略（局部地图 + 全局地图）
    4. 实时可视化
    5. ROS2消息发布
    """
    
    def __init__(self):
        """初始化高程图生成器节点"""
        super().__init__('elevation_map_generator')
        
        # ========== 局部地图参数配置 ==========
        # 局部地图：高分辨率，小范围，跟随机器人移动
        self.declare_parameter('local_grid_size', 0.1)    # 局部地图网格大小：10cm
        self.declare_parameter('local_map_width', 1.0)    # 局部地图宽度：1.0米（Y方向）
        self.declare_parameter('local_map_height', 1.6)   # 局部地图长度：1.6米（X方向）
        
        # 获取参数值
        self.local_grid_size = self.get_parameter('local_grid_size').get_parameter_value().double_value
        self.local_map_width = self.get_parameter('local_map_width').get_parameter_value().double_value
        self.local_map_height = self.get_parameter('local_map_height').get_parameter_value().double_value
        
        # 创建局部地图的网格坐标系（用于3D可视化）
        # X方向：从-height/2 到 +height/2
        x_points = np.arange(-self.local_map_height, self.local_map_height + self.local_grid_size, self.local_grid_size)
        # Y方向：从-width/2 到 +width/2  
        y_points = np.arange(-self.local_map_width, self.local_map_width + self.local_grid_size, self.local_grid_size)
        # 创建网格坐标矩阵
        self.local_grid = np.meshgrid(x_points, y_points)
        # 计算网格维度
        self.local_grid_width = len(x_points)    # X方向网格数量
        self.local_grid_height = len(y_points)   # Y方向网格数量

        # ========== 全局地图参数配置 ==========
        # 全局地图：中等分辨率，大范围，固定坐标系
        self.declare_parameter('global_grid_size', 0.02)   # 全局地图网格大小：m
        self.declare_parameter('global_map_width', 5.0)   # 全局地图宽度：米
        self.declare_parameter('global_map_height', 5.0)  # 全局地图长度：米
        
        # 获取参数值
        self.global_grid_size = self.get_parameter('global_grid_size').get_parameter_value().double_value
        self.global_map_width = self.get_parameter('global_map_width').get_parameter_value().double_value
        self.global_map_height = self.get_parameter('global_map_height').get_parameter_value().double_value
        
        # 计算全局地图网格维度
        self.global_grid_width = int(self.global_map_width / self.global_grid_size)
        self.global_grid_height = int(self.global_map_height / self.global_grid_size)
        
        # 设置全局地图原点（地图中心为原点）
        self.global_map_origin_x = -self.global_map_width / 2.0
        self.global_map_origin_y = -self.global_map_height / 2.0

        # ========== 其他系统参数 ==========
        self.declare_parameter('data_timeout_sec', 300.0)      # 数据超时时间：5分钟
        self.declare_parameter('min_points_per_cell', 1)        # 网格有效的最小点数
        self.declare_parameter('publish_global_map', True)      # 是否发布全局地图
        
        # 获取参数值
        self.data_timeout_sec = self.get_parameter('data_timeout_sec').get_parameter_value().double_value
        self.min_points_per_cell = self.get_parameter('min_points_per_cell').get_parameter_value().integer_value
        self.publish_global_map = self.get_parameter('publish_global_map').get_parameter_value().bool_value
        
        # ========== 高程基准点追踪 ==========
        # 追踪全局最低点作为高程基准
        self.global_lowest_elevation = float('inf')  # 初始化为正无穷，第一个点会更新它
        self.elevation_reference_updated = False     # 标记基准点是否已更新
        
        # ========== 数据存储初始化 ==========
        # 局部地图：使用2D numpy数组，密集存储
        self.local_elevation_map = np.zeros((self.local_grid_height, self.local_grid_width), dtype=np.float32)
        
        # 全局地图：使用哈希表，稀疏存储（只存储有数据的网格）
        # 键：(grid_x, grid_y) 网格坐标
        # 值：GlobalGridCell 网格单元对象
        self.global_elevation_map: Dict[Tuple[int, int], GlobalGridCell] = {}
        
        # 机器人位姿信息
        self.current_pose: Optional[Odometry] = None    # 当前机器人位姿
        self.has_odom = False                           # 是否接收到里程计数据
        
        # ========== ROS2订阅者创建 ==========
        # 订阅激光雷达点云数据（经过SLAM处理的全局坐标系点云）
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/cloud_registered',  # Fast-LIO2发布的配准后点云（全局坐标系）
            self.cloud_callback,
            10
        )
        
        # 订阅机器人里程计数据（SLAM输出的优化后位姿）
        self.odom_sub = self.create_subscription(
            Odometry,
            '/aft_mapped_to_init',  # Fast-LIO2发布的优化后里程计
            self.odom_callback,
            10
        )
        
        # ========== ROS2发布者创建 ==========
        # 发布局部高程图（OccupancyGrid格式）
        self.local_elevation_pub = self.create_publisher(
            OccupancyGrid,
            '/elevation_map_local',
            10
        )
        
        # 发布全局高程图（如果启用）
        if self.publish_global_map:
            self.global_elevation_pub = self.create_publisher(
                OccupancyGrid,
                '/elevation_map_global',
                10
            )
        
        # ========== 定时器创建 ==========
        # 局部地图更新定时器：10Hz，实时性要求高
        self.local_timer = self.create_timer(0.1, self.get_local_elevation_map)
        
        # 全局地图发布定时器：1Hz，减少计算负担
        if self.publish_global_map:
            self.global_timer = self.create_timer(1.0, self.publish_global_elevation_map)
        
        # 数据清理定时器：每30秒清理一次过期数据
        self.cleanup_timer = self.create_timer(30.0, self.cleanup_old_data)
        
        # ========== 日志信息输出 ==========
        self.get_logger().info('ElevationMapGenerator initialized')
        self.get_logger().info(f'Local: {self.local_grid_size:.2f} m grid, {self.local_map_width:.1f} x {self.local_map_height:.1f} m area')
        self.get_logger().info(f'Global: {self.global_grid_size:.2f} m grid, {self.global_map_width:.1f} x {self.global_map_height:.1f} m map')

        # ========== 可视化系统初始化 ==========
        self.global_map_data = None  # 全局地图数据缓存
        self.local_map_data = None   # 局部地图数据缓存
        
        # 打印网格配置信息
        self.get_logger().info(f'Global grid: {self.global_grid_width} x {self.global_grid_height}, size: {self.global_grid_size}')
        self.get_logger().info(f'Local grid: {self.local_grid_width} x {self.local_grid_height}, size: {self.local_grid_size}')
        
        # 创建3D可视化器
        # 全局地图：更新频率1/20 = 0.05Hz（20秒更新一次）
        self.global_height_plotter = HeightMapPlotter(self.global_grid_width, self.global_grid_height, self.global_grid_size, 1.0/20, "Global Elevation Map")
        # 局部地图：更新频率0.1/20 = 0.005Hz（200秒更新一次，实际会更频繁调用）
        self.local_height_plotter = HeightMapPlotter(self.local_grid_width, self.local_grid_height, self.local_grid_size, 0.1/20, "Local Elevation Map")
        
        # 配置机器人坐标系显示
        # 全局地图中使用更大的坐标轴，确保俯视时也能看到
        self.global_height_plotter.set_axis_length(0.3)  # 米
        self.global_height_plotter.set_show_robot_frame(True)
        
        # 局部地图中使用较小的坐标轴
        self.local_height_plotter.set_axis_length(0.3)
        self.local_height_plotter.set_show_robot_frame(True)
        
        # 启用基准点信息显示
        self.global_height_plotter.show_reference_info = True
        self.local_height_plotter.show_reference_info = True

        # 启用matplotlib交互模式，支持实时显示
        plt.ion()
        plt.show(block=False)

    def odom_callback(self, msg: Odometry):
        """
        里程计数据回调函数
        
        Args:
            msg (Odometry): 包含机器人位姿信息的里程计消息
            
        该函数接收SLAM系统输出的优化后机器人位姿，
        用于局部地图的坐标系转换。
        """
        self.current_pose = msg
        self.has_odom = True
    
    def cloud_callback(self, msg: PointCloud2):
        """
        点云数据回调函数
        
        Args:
            msg (PointCloud2): 激光雷达点云消息
            
        该函数是整个系统的核心处理函数，负责：
        1. 解析点云二进制数据
        2. 验证数据有效性
        3. 更新全局高程图
        4. 记录处理统计信息
        """
        # 确保已接收到机器人位姿信息
        if not self.has_odom:
            self.get_logger().warn('No odometry data received yet', throttle_duration_sec=2.0)
            return
        
        try:
            # 解析PointCloud2消息，提取XYZ坐标
            points_GCS = self.parse_pointcloud2(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to convert point cloud: {e}', throttle_duration_sec=5.0)
            return
        
        # 获取当前时间戳，用于数据时效性管理
        current_time = self.get_clock().now()
        
        # 逐点处理点云数据
        valid_points = 0
        for point in points_GCS:
            x_gcs, y_gcs, z_gcs = point
            
            # 数据有效性检查：过滤NaN和无穷大值
            if not (math.isfinite(x_gcs) and math.isfinite(y_gcs) and math.isfinite(z_gcs)):
                continue
            
            # 更新全局高程图（使用世界坐标系）
            self.update_global_map(x_gcs, y_gcs, z_gcs, current_time)
            valid_points += 1
        
        # 输出处理统计信息
        self.get_logger().debug(f'Processed {valid_points} points, global map size: {len(self.global_elevation_map)}')
    
    def parse_pointcloud2(self, msg: PointCloud2):
        """
        解析PointCloud2消息的二进制数据
        
        Args:
            msg (PointCloud2): ROS2点云消息
            
        Returns:
            List[Tuple[float, float, float]]: 解析后的(x, y, z)坐标列表
            
        PointCloud2使用二进制格式存储点云数据，需要根据字段信息
        解析每个点的坐标。该函数处理了字段偏移、数据类型转换等细节。
        """
        points = []
        
        # 构建字段名到字段信息的映射
        fields = {field.name: field for field in msg.fields}
        
        # 验证必需的坐标字段是否存在
        if 'x' not in fields or 'y' not in fields or 'z' not in fields:
            raise ValueError("PointCloud2 message missing x, y, or z fields")
        
        # 获取各坐标字段在点数据中的字节偏移量
        x_offset = fields['x'].offset
        y_offset = fields['y'].offset
        z_offset = fields['z'].offset
        point_step = msg.point_step  # 每个点的字节长度

        data = msg.data
        
        # 逐点解析二进制数据
        for i in range(0, len(data), point_step):
            # 防止数据截断
            if i + point_step > len(data): 
                break
            
            # 使用struct解包32位浮点数（'f'表示float格式）
            x = struct.unpack('f', data[i + x_offset:i + x_offset + 4])[0]
            y = struct.unpack('f', data[i + y_offset:i + y_offset + 4])[0]
            z = struct.unpack('f', data[i + z_offset:i + z_offset + 4])[0]
            
            points.append((x, y, z))
        return points
    
    def quaternion_to_yaw(self, quat: Quaternion) -> float:
        """
        将四元数转换为yaw角（绕Z轴旋转角）
        
        Args:
            quat (Quaternion): 四元数表示的旋转
            
        Returns:
            float: yaw角度（弧度制）
            
        机器人通常只关心平面内的朝向，因此只需要yaw角。
        该函数使用标准的四元数到欧拉角转换公式。
        """
        # 四元数到yaw角的转换公式
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def global_index_to_coordinate(self, grid_x, grid_y):
        """
        将全局地图网格索引转换为世界坐标
        
        Args:
            grid_x (int): 网格X索引
            grid_y (int): 网格Y索引
            
        Returns:
            Tuple[float, float]: 对应的世界坐标(x, y)
        """
        x_gcs = grid_x * self.global_grid_size + self.global_map_origin_x
        y_gcs = grid_y * self.global_grid_size + self.global_map_origin_y
        return x_gcs, y_gcs

    def global_coordinate_to_index(self, x_gcs, y_gcs):
        """
        将世界坐标转换为全局地图网格索引
        
        Args:
            x_gcs (float): 世界坐标X
            y_gcs (float): 世界坐标Y
            
        Returns:
            Tuple[int, int]: 对应的网格索引(grid_x, grid_y)
        """
        grid_x = int((x_gcs - self.global_map_origin_x) / self.global_grid_size)
        grid_y = int((y_gcs - self.global_map_origin_y) / self.global_grid_size)        
        return grid_x, grid_y
    
    def local_index_to_coordinate(self, grid_x, grid_y):
        """
        将局部地图网格索引转换为机器人坐标系坐标
        
        Args:
            grid_x (int): 局部网格X索引
            grid_y (int): 局部网格Y索引
            
        Returns:
            Tuple[float, float]: 机器人坐标系下的坐标(x, y)
        """
        x_bcs = grid_x * self.local_grid_size - self.local_map_height/2
        y_bcs = grid_y * self.local_grid_size - self.local_map_width/2
        return x_bcs, y_bcs

    def local_coordinate_to_index(self, x_bcs, y_bcs):
        """
        将机器人坐标系坐标转换为局部地图网格索引
        
        Args:
            x_bcs (float): 机器人坐标系X坐标
            y_bcs (float): 机器人坐标系Y坐标
            
        Returns:
            Tuple[int, int]: 对应的局部网格索引(grid_x, grid_y)
        """
        grid_x = int((x_bcs + self.local_map_height/2) / self.local_grid_size)
        grid_y = int((y_bcs + self.local_map_width/2) / self.local_grid_size)
        return grid_x, grid_y

    def update_global_map(self, world_x: float, world_y: float, z: float, time: Time):
        """
        更新全局高程图
        
        Args:
            world_x (float): 世界坐标系X坐标
            world_y (float): 世界坐标系Y坐标
            z (float): 高程值
            time (Time): 时间戳
            
        全局地图使用稀疏哈希表存储，只为有数据的网格创建存储单元。
        这种设计在大范围环境中能显著节省内存。
        """
        # 边界检查：确保点在全局地图范围内
        if (world_x < self.global_map_origin_x or 
            world_x >= self.global_map_origin_x + self.global_map_width or
            world_y < self.global_map_origin_y or 
            world_y >= self.global_map_origin_y + self.global_map_height):
            return
        
        # 更新全局最低点基准
        if z < self.global_lowest_elevation:
            self.global_lowest_elevation = z
            self.elevation_reference_updated = True
            self.get_logger().info(f'New lowest elevation: {z:.3f}m')
            
            # 更新可视化器的基准点信息
            self.global_height_plotter.set_elevation_reference(z)
            self.local_height_plotter.set_elevation_reference(z)
        
        # 世界坐标转换为网格索引
        grid_x, grid_y = self.global_coordinate_to_index(world_x, world_y)
        key = (grid_x, grid_y)
        
        # 惰性创建：只在需要时创建网格单元
        if key not in self.global_elevation_map:
            self.global_elevation_map[key] = GlobalGridCell()
        
        # 向网格单元添加新的高程观测
        self.global_elevation_map[key].add_point(z, time)
    
    def get_local_elevation_map(self):
        """
        生成局部高程图
        
        该函数从全局地图中提取机器人周围的高程数据，
        转换到机器人坐标系，生成局部高精度地图。
        
        处理流程：
        1. 遍历全局地图中的所有有效网格
        2. 将网格坐标转换为世界坐标
        3. 世界坐标转换为机器人坐标系
        4. 机器人坐标转换为局部网格索引
        5. 累积高程值到局部地图
        """
        # 确保有位姿信息
        if not self.has_odom: 
            return

        # 初始化局部地图数据结构
        # 使用与meshgrid相同的维度： (Y方向, X方向) = (height, width)
        elevation_map = np.zeros((self.local_grid_height, self.local_grid_width), dtype=float)
        point_num = np.zeros((self.local_grid_height, self.local_grid_width), dtype=float)
        
        # 获取当前机器人位姿
        robot_x = self.current_pose.pose.pose.position.x
        robot_y = self.current_pose.pose.pose.position.y
        orientation = self.current_pose.pose.pose.orientation
        robot_yaw = self.quaternion_to_yaw(orientation)
        
        # 统计变量
        valid_local_points = 0
        total_global_points = 0
        
        # 遍历全局地图中的所有网格单元
        for (global_grid_x, global_grid_y), cell in self.global_elevation_map.items():
            # 过滤：只处理有足够数据点的网格
            if cell.is_valid() and cell.point_count >= self.min_points_per_cell:
                total_global_points += 1

                # 网格索引转换为世界坐标
                x_gcs, y_gcs = self.global_index_to_coordinate(global_grid_x, global_grid_y)

                # 世界坐标转换为机器人坐标系
                # 1. 平移：将世界坐标原点移到机器人位置
                dx = x_gcs - robot_x
                dy = y_gcs - robot_y
                # 2. 旋转：消除机器人朝向影响，使X轴指向机器人前方
                local_x = dx * math.cos(-robot_yaw) - dy * math.sin(-robot_yaw)
                local_y = dx * math.sin(-robot_yaw) + dy * math.cos(-robot_yaw)
                
                # 机器人坐标转换为局部网格索引
                local_grid_x, local_grid_y = self.local_coordinate_to_index(local_x, local_y)
                
                # 边界检查：跳过超出局部地图范围的点
                if (local_grid_x < 0 or local_grid_x >= self.local_grid_width or 
                    local_grid_y < 0 or local_grid_y >= self.local_grid_height):
                    continue  # 使用continue而不是return，确保处理所有点
                
                # 累积高程值和点数（转换为相对于最低点的高程）
                # 只有在已经找到最低点时才进行相对高程计算
                if self.global_lowest_elevation != float('inf'):
                    relative_elevation = cell.avg_elevation - self.global_lowest_elevation
                    # 调试：记录一些样本的相对高程计算
                    if valid_local_points < 3:  # 只记录前几个点
                        print(f"DEBUG elevation calc: cell.avg={cell.avg_elevation:.6f}, lowest={self.global_lowest_elevation:.6f}, relative={relative_elevation:.6f}")
                else:
                    relative_elevation = cell.avg_elevation  # 还没找到最低点时使用绝对高程
                    if valid_local_points < 3:  # 只记录前几个点
                        print(f"DEBUG elevation calc (no lowest): cell.avg={cell.avg_elevation:.6f}, relative={relative_elevation:.6f}")
                
                # 注意：numpy数组索引是[row, col] = [y, x]
                elevation_map[local_grid_y, local_grid_x] += relative_elevation
                point_num[local_grid_y, local_grid_x] += 1
                valid_local_points += 1
        
        # 计算平均高程
        # 只对有数据的网格计算平均值，无数据的网格保持为0
        self.local_map_data = np.zeros((self.local_grid_height, self.local_grid_width), dtype=float)
        
        # 对有数据的网格计算平均高程
        valid_mask = point_num > 0
        
        # 调试：检查累积值
        valid_grids = np.sum(valid_mask)
        if valid_grids > 0:
            print(f"DEBUG: Found {valid_grids} grids with data")
            print(f"DEBUG: elevation_map range: [{np.min(elevation_map[valid_mask]):.6f}, {np.max(elevation_map[valid_mask]):.6f}]")
            print(f"DEBUG: point_num range: [{np.min(point_num[valid_mask]):.1f}, {np.max(point_num[valid_mask]):.1f}]")
            
            # 检查前几个有效网格的详细计算
            valid_indices = np.where(valid_mask)
            for i in range(min(3, len(valid_indices[0]))):
                row, col = valid_indices[0][i], valid_indices[1][i]
                elev_sum = elevation_map[row, col]
                point_count = point_num[row, col]
                avg_result = elev_sum / point_count
                print(f"DEBUG: Grid[{row},{col}]: sum={elev_sum:.6f}, count={point_count:.1f}, avg={avg_result:.6f}")
        
        self.local_map_data[valid_mask] = elevation_map[valid_mask] / point_num[valid_mask]
        
        # 调试：检查计算结果
        if valid_grids > 0:
            print(f"DEBUG: After division, local_map_data range: [{np.min(self.local_map_data[valid_mask]):.6f}, {np.max(self.local_map_data[valid_mask]):.6f}]")
        
        # 更新可视化数据
        self.local_height_plotter.height_map = self.local_map_data
        
        # 更新机器人位姿信息到可视化器（局部地图中机器人始终在中心）
        # 局部地图的显示坐标系：
        # - 原点在局部地图的左下角
        # - X轴向右（对应机器人坐标系经过旋转后的X轴）
        # - Y轴向上（对应机器人坐标系经过旋转后的Y轴）
        
        # 机器人在局部地图显示坐标系中的位置（理论上在中心，但可以微调以提高精度）
        # 由于局部地图是离散网格，机器人实际位置可能不完全在网格中心
        robot_display_x = self.local_map_height / 2.0  # 局部地图中心X坐标（米）
        robot_display_y = self.local_map_width / 2.0   # 局部地图中心Y坐标（米）
        
        # 可选：根据网格分辨率进行微调，使机器人位置更精确地对应网格中心
        # grid_center_x = (self.local_grid_width // 2) * self.local_grid_size + self.local_grid_size / 2.0
        # grid_center_y = (self.local_grid_height // 2) * self.local_grid_size + self.local_grid_size / 2.0
        # robot_display_x = grid_center_x
        # robot_display_y = grid_center_y
        
        # 机器人在局部地图中的高度：相对于最低点
        if self.has_odom and self.global_lowest_elevation != float('inf'):
            robot_world_z = self.current_pose.pose.pose.position.z
            robot_display_z = robot_world_z - self.global_lowest_elevation
            
            # 尝试从局部地图数据中获取机器人位置的高程（更精确的地面高度）
            if valid_local_points > 0:
                # 获取机器人位置对应的网格索引
                center_grid_x = self.local_grid_width // 2
                center_grid_y = self.local_grid_height // 2
                
                # 检查机器人位置周围的高程数据
                if (0 <= center_grid_y < self.local_map_data.shape[0] and 
                    0 <= center_grid_x < self.local_map_data.shape[1]):
                    local_ground_height = self.local_map_data[center_grid_y, center_grid_x]
                    
                    # 如果有有效的地面高程数据，将机器人稍微抬高一点显示在地面上方
                    if np.isfinite(local_ground_height) and local_ground_height != 0:
                        robot_display_z = local_ground_height + 0.1  # 地面上方10cm
                        
        else:
            robot_display_z = 0.0  # 默认高度
        
        # 机器人在局部地图中的朝向：
        # 在局部地图坐标系中，X轴始终指向机器人前方，所以显示朝向为0
        # 但为了更好地反映旋转，我们可以显示实际的yaw角度
        robot_display_yaw = 0.0  # 在局部坐标系中，机器人朝向固定为X正方向
        
        # 设置机器人位姿到局部地图可视化器
        self.local_height_plotter.set_robot_pose(robot_display_x, robot_display_y, robot_display_z, robot_display_yaw)
        
        # 输出调试信息
        if valid_local_points > 0:
            print(f"Local robot display: pos({robot_display_x:.3f}, {robot_display_y:.3f}, {robot_display_z:.3f}), yaw: {robot_display_yaw:.3f}")
            print(f"World robot pose: pos({robot_x:.3f}, {robot_y:.3f}, {self.current_pose.pose.pose.position.z:.3f}), yaw: {robot_yaw:.3f}")
        
        # 输出调试信息
        self.get_logger().debug(f'Local map: {total_global_points} global points, {valid_local_points} valid local points')
        if valid_local_points == 0:
            self.get_logger().warn(f'No valid points in local map! Global map has {len(self.global_elevation_map)} cells, min_points_per_cell={self.min_points_per_cell}')
            
            # 输出更详细的调试信息
            print(f"Robot pose: ({robot_x:.3f}, {robot_y:.3f}), yaw: {robot_yaw:.3f}")
            print(f"Local map range: X=[{-self.local_map_height/2:.3f}, {self.local_map_height/2:.3f}], Y=[{-self.local_map_width/2:.3f}, {self.local_map_width/2:.3f}]")
            print(f"Local grid dimensions: width={self.local_grid_width} (X), height={self.local_grid_height} (Y)")
            print(f"Global map origin: ({self.global_map_origin_x:.3f}, {self.global_map_origin_y:.3f})")
            print(f"Global map size: {self.global_map_width}x{self.global_map_height}")
            print(f"Global lowest elevation: {self.global_lowest_elevation}")
            
            # 检查前几个全局网格是否在范围内
            sample_count = 0
            for (global_grid_x, global_grid_y), cell in self.global_elevation_map.items():
                if sample_count >= 5:
                    break
                if cell.is_valid() and cell.point_count >= self.min_points_per_cell:
                    x_gcs, y_gcs = self.global_index_to_coordinate(global_grid_x, global_grid_y)
                    dx = x_gcs - robot_x
                    dy = y_gcs - robot_y
                    local_x = dx * math.cos(-robot_yaw) - dy * math.sin(-robot_yaw)
                    local_y = dx * math.sin(-robot_yaw) + dy * math.cos(-robot_yaw)
                    local_grid_x, local_grid_y = self.local_coordinate_to_index(local_x, local_y)
                    
                    print(f"Sample {sample_count}: global({global_grid_x}, {global_grid_y}) -> world({x_gcs:.3f}, {y_gcs:.3f}) -> local({local_x:.3f}, {local_y:.3f}) -> grid({local_grid_x}, {local_grid_y})")
                    print(f"  In bounds: {0 <= local_grid_x < self.local_grid_width and 0 <= local_grid_y < self.local_grid_height}")
                    sample_count += 1
        
        # 详细的局部地图数据分析
        print(f"=== Local Map Data Analysis ===")
        print(f"Shape: {self.local_map_data.shape}")
        print(f"Valid points added: {valid_local_points}")
        print(f"Total elements: {self.local_map_data.size}")
        
        # 分析数据分布
        finite_data = self.local_map_data[np.isfinite(self.local_map_data)]
        nonzero_data = finite_data[finite_data != 0]
        zero_data = finite_data[finite_data == 0]
        
        print(f"Finite data points: {len(finite_data)}")
        print(f"Zero data points: {len(zero_data)}")
        print(f"Non-zero data points: {len(nonzero_data)}")
        
        if len(finite_data) > 0:
            print(f"Data range: [{np.min(finite_data):.6f}, {np.max(finite_data):.6f}]")
            print(f"Data mean: {np.mean(finite_data):.6f}")
            print(f"Data std: {np.std(finite_data):.6f}")
        
        if len(nonzero_data) > 0:
            print(f"Non-zero range: [{np.min(nonzero_data):.6f}, {np.max(nonzero_data):.6f}]")
            print(f"Non-zero mean: {np.mean(nonzero_data):.6f}")
            
            # 显示一些具体的非零值样本
            sample_indices = np.where(self.local_map_data != 0)
            if len(sample_indices[0]) > 0:
                print("Sample non-zero values:")
                for i in range(min(5, len(sample_indices[0]))):
                    row, col = sample_indices[0][i], sample_indices[1][i]
                    val = self.local_map_data[row, col]
                    print(f"  [{row}, {col}] = {val:.6f}")
        else:
            print("No non-zero data found - all valid data is exactly 0.0")
            
        print(f"===============================")
        
        # 检查点数统计是否合理
        print(f"Point accumulation analysis:")
        nonzero_point_count = np.sum(point_num > 0)
        print(f"Grids with data: {nonzero_point_count}")
        if nonzero_point_count > 0:
            max_points = np.max(point_num)
            min_points = np.min(point_num[point_num > 0])
            print(f"Points per grid range: [{min_points:.0f}, {max_points:.0f}]")
        
        # 额外的调试信息，即使有有效点也输出
        if valid_local_points > 0:
            print(f"SUCCESS: Found {valid_local_points} valid points in local map")
            print(f"Robot pose: ({robot_x:.3f}, {robot_y:.3f}), yaw: {robot_yaw:.3f}")
            print(f"Local map data range: [{np.min(self.local_map_data):.3f}, {np.max(self.local_map_data):.3f}]")
            
            # 检查前几个有效点的坐标转换
            sample_count = 0
            for (global_grid_x, global_grid_y), cell in self.global_elevation_map.items():
                if sample_count >= 3:
                    break
                if cell.is_valid() and cell.point_count >= self.min_points_per_cell:
                    x_gcs, y_gcs = self.global_index_to_coordinate(global_grid_x, global_grid_y)
                    dx = x_gcs - robot_x
                    dy = y_gcs - robot_y
                    local_x = dx * math.cos(-robot_yaw) - dy * math.sin(-robot_yaw)
                    local_y = dx * math.sin(-robot_yaw) + dy * math.cos(-robot_yaw)
                    local_grid_x, local_grid_y = self.local_coordinate_to_index(local_x, local_y)
                    
                    if (0 <= local_grid_x < self.local_grid_width and 0 <= local_grid_y < self.local_grid_height):
                        print(f"Valid point {sample_count}: global({global_grid_x}, {global_grid_y}) -> world({x_gcs:.3f}, {y_gcs:.3f}) -> local({local_x:.3f}, {local_y:.3f}) -> grid({local_grid_x}, {local_grid_y})")
                        if self.global_lowest_elevation != float('inf'):
                            relative_elevation = cell.avg_elevation - self.global_lowest_elevation
                        else:
                            relative_elevation = cell.avg_elevation
                        print(f"  Elevation: {cell.avg_elevation:.3f} -> relative: {relative_elevation:.3f}")
                        sample_count += 1
        
        # 更新3D可视化
        self.local_height_plotter.update_plot()
        return self.local_map_data

    def publish_global_elevation_map(self):
        """
        发布全局高程图
        
        该函数将稀疏的全局地图转换为稠密的OccupancyGrid格式，
        用于ROS2系统中的导航和可视化。
        
        处理流程：
        1. 创建稠密数组（初始化为-inf）
        2. 从哈希表填充有效数据
        3. 归一化到0-100范围
        4. 发布ROS2消息
        5. 更新可视化
        """
        # 创建ROS2 OccupancyGrid消息
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'  # 使用固定的地图坐标系
        
        # 设置地图元信息
        grid_msg.info.resolution = self.global_grid_size
        grid_msg.info.width = self.global_grid_width
        grid_msg.info.height = self.global_grid_height
        
        # 设置地图原点（固定不变）
        grid_msg.info.origin.position.x = self.global_map_origin_x
        grid_msg.info.origin.position.y = self.global_map_origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        # 创建稠密数组：将稀疏哈希表转换为稠密格式
        global_map_data = np.full(
            self.global_grid_width * self.global_grid_height,
            float('-inf'),  # 初始化为无效值
            dtype=np.float32
        )
        
        # 统计变量
        valid_cells_in_bounds = 0
        total_valid_cells = 0
        
        # 从哈希表填充稠密数组
        for (grid_x, grid_y), cell in self.global_elevation_map.items():
            if cell.is_valid() and cell.point_count >= self.min_points_per_cell:
                total_valid_cells += 1
                
                # 边界检查：确保网格索引在有效范围内
                if (0 <= grid_x < self.global_grid_width and 
                    0 <= grid_y < self.global_grid_height):
                    # 二维索引转换为一维索引（行优先）
                    index = grid_y * self.global_grid_width + grid_x
                    # 转换为相对于最低点的高程
                    if self.global_lowest_elevation != float('inf'):
                        relative_elevation = cell.avg_elevation - self.global_lowest_elevation
                    else:
                        relative_elevation = cell.avg_elevation  # 还没找到最低点时使用绝对高程
                    global_map_data[index] = relative_elevation
                    valid_cells_in_bounds += 1
        
        # 输出调试信息
        self.get_logger().debug(f'Global map: {total_valid_cells} valid cells, {valid_cells_in_bounds} in bounds')
        if valid_cells_in_bounds == 0:
            self.get_logger().warn(f'No valid cells in global map bounds! Total global cells: {len(self.global_elevation_map)}, valid: {total_valid_cells}')
            self.get_logger().warn(f'Global map bounds: width={self.global_grid_width}, height={self.global_grid_height}')
            
            # 输出超出边界的样本索引（用于调试）
            out_of_bounds_samples = []
            for (grid_x, grid_y), cell in list(self.global_elevation_map.items())[:5]:
                if cell.is_valid() and cell.point_count >= self.min_points_per_cell:
                    if not (0 <= grid_x < self.global_grid_width and 0 <= grid_y < self.global_grid_height):
                        out_of_bounds_samples.append((grid_x, grid_y))
            
            if out_of_bounds_samples:
                self.get_logger().warn(f'Sample out-of-bounds indices: {out_of_bounds_samples}')
        
        # 准备可视化数据：将一维数组重塑为二维（注意行列顺序）
        self.global_map_data = global_map_data.reshape(self.global_grid_height, self.global_grid_width)
        
        # 归一化并填充ROS2消息数据
        self.normalize_and_fill_grid(global_map_data, grid_msg)
        
        # 统计并输出数据信息
        valid_count = np.sum(self.global_map_data != float('-inf'))
        if valid_count > 0:
            valid_data = self.global_map_data[self.global_map_data != float('-inf')]
            min_val = np.min(valid_data)
            max_val = np.max(valid_data)
            mean_val = np.mean(valid_data)
            print(f"global: {self.global_map_data.shape}, valid_points: {valid_count}, range: [{min_val:.3f}, {max_val:.3f}], mean: {mean_val:.3f}")
        else:
            print(f"global: {self.global_map_data.shape}, valid_points: 0, all data is -inf")
            
        # 处理可视化数据：将-inf替换为0以便正常显示（相对于最低点）
        display_data = self.global_map_data.copy()
        display_data[display_data == float('-inf')] = 0.0
        self.global_height_plotter.height_map = display_data
        
        # 更新机器人位姿信息到全局地图可视化器
        if self.has_odom:
            # 获取机器人在世界坐标系中的位姿
            robot_world_x = self.current_pose.pose.pose.position.x
            robot_world_y = self.current_pose.pose.pose.position.y
            robot_world_z = self.current_pose.pose.pose.position.z
            orientation = self.current_pose.pose.pose.orientation
            robot_yaw = self.quaternion_to_yaw(orientation)
            
            # 转换到全局地图显示坐标系
            # 将世界坐标转换为显示坐标（相对于地图原点的偏移）
            display_x = robot_world_x - self.global_map_origin_x
            display_y = robot_world_y - self.global_map_origin_y
            
            # 使用相对于最低点的高度
            if self.global_lowest_elevation != float('inf'):
                display_z = robot_world_z - self.global_lowest_elevation
            else:
                display_z = robot_world_z
            
            # 设置机器人位姿到全局地图可视化器
            self.global_height_plotter.set_robot_pose(display_x, display_y, display_z, robot_yaw)
            
            # 输出调试信息
            if valid_count > 0:  # 只在有有效数据时输出
                print(f"Global robot display: pos({display_x:.3f}, {display_y:.3f}, {display_z:.3f}), yaw: {robot_yaw:.3f} ({math.degrees(robot_yaw):.1f}°)")
                print(f"Global map bounds: X=[{self.global_map_origin_x:.3f}, {self.global_map_origin_x + self.global_map_width:.3f}], Y=[{self.global_map_origin_y:.3f}, {self.global_map_origin_y + self.global_map_height:.3f}]")
                
                # 检查机器人是否在地图边界内
                in_bounds_x = 0 <= display_x <= self.global_map_width
                in_bounds_y = 0 <= display_y <= self.global_map_height
                print(f"Robot in bounds: X={in_bounds_x}, Y={in_bounds_y}")
            
        # 更新3D可视化
        self.global_height_plotter.update_plot()

        # 发布ROS2消息
        self.global_elevation_pub.publish(grid_msg)
        
        self.get_logger().debug(f'Published global map: {grid_msg.info.width}x{grid_msg.info.height}, {len(self.global_elevation_map)} known cells')
    
    def normalize_and_fill_grid(self, elevation_data: np.ndarray, grid_msg: OccupancyGrid):
        """
        归一化高程数据并填充到OccupancyGrid消息
        
        Args:
            elevation_data (np.ndarray): 原始高程数据数组
            grid_msg (OccupancyGrid): 要填充的ROS2消息
            
        OccupancyGrid使用0-100表示占用概率，-1表示未知。
        该函数将任意范围的高程值映射到这个标准化范围。
        """
        # 初始化消息数据数组
        grid_msg.data = [0] * len(elevation_data)
        
        # 提取有效的高程值（过滤-inf）
        valid_elevations = elevation_data[elevation_data != float('-inf')]
        
        self.get_logger().debug(f'normalize_and_fill_grid: {len(valid_elevations)} valid elevations out of {len(elevation_data)} total')
        
        # 处理无有效数据的情况
        if len(valid_elevations) == 0:
            grid_msg.data = [-1] * len(elevation_data)  # 全部设为未知
            self.get_logger().warn('No valid elevations found - setting all to unknown (-1)')
            return
        
        # 计算高程范围
        min_elevation = np.min(valid_elevations)
        max_elevation = np.max(valid_elevations)
        
        self.get_logger().debug(f'Elevation range: {min_elevation:.2f} to {max_elevation:.2f}')
        
        # 逐元素归一化到0-100范围
        for i, elevation in enumerate(elevation_data):
            if elevation == float('-inf'):
                grid_msg.data[i] = -1  # 未知区域
            else:
                if max_elevation > min_elevation:
                    # 线性归一化：(值-最小值)/(最大值-最小值) * 100
                    normalized = (elevation - min_elevation) / (max_elevation - min_elevation)
                    grid_msg.data[i] = int(normalized * 100)
                else:
                    # 所有值相同：设为中等值
                    grid_msg.data[i] = 50
    
    def cleanup_old_data(self):
        """
        清理全局地图中的过期数据
        
        该函数定期执行，删除超过指定时间未更新的网格单元，
        防止内存无限增长，适合长时间运行的系统。
        """
        # 检查是否启用数据清理
        if self.data_timeout_sec <= 0:
            return
        
        current_time = self.get_clock().now()
        timeout = Duration(seconds=self.data_timeout_sec)
        
        # 收集需要删除的网格键
        keys_to_remove = []
        for key, cell in self.global_elevation_map.items():
            if current_time - cell.last_update > timeout:
                keys_to_remove.append(key)
        
        # 批量删除过期数据
        for key in keys_to_remove:
            del self.global_elevation_map[key]
        
        self.get_logger().debug(f'Cleanup: global map now has {len(self.global_elevation_map)} cells')


def main(args=None):
    """
    主函数：启动高程图生成器节点
    
    Args:
        args: 命令行参数
        
    该函数初始化ROS2系统，创建节点实例，并启动事件循环。
    包含适当的异常处理和资源清理。
    """
    # 初始化ROS2系统
    rclpy.init(args=args)
    
    # 创建高程图生成器节点
    node = ElevationMapGenerator()
    
    try:
        # 启动ROS2事件循环（阻塞运行）
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 处理Ctrl+C中断
        pass
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()