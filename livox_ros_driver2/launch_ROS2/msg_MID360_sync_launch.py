import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Start the sync_node with optimized parameters
    sync_node = Node(
        package='livox_ros_driver2',
        executable='sync_node',
        name='sync_node',
        output='screen',
        parameters=[{
            'image_topic': '/camera/camera/color/image_raw',
            'lidar_topic': '/livox/lidar',
            'synced_image_topic': '/synced_image',
            'synced_lidar_topic': '/synced_lidar',
            'publish_rate': 10.0,  # 降低到10Hz提高稳定性
            'max_time_diff': 0.020,  # 增加到20ms容差
            'queue_size': 5,  # 增加队列大小
            'sync_slop': 0.010  # 10ms同步滑动窗口
        }]
    )

    return LaunchDescription([
        sync_node
    ]) 