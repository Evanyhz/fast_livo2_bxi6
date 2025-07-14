#!/usr/bin/python3
# -- coding: utf-8 --**

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    
    # Find path
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "mapping_view.rviz")

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen"
        ),
    ]) 