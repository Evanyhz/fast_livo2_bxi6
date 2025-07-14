#!/usr/bin/python3
# -- coding: utf-8 --**

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    
    # Find path
    config_file_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "M300.rviz")

    # Load parameters - 使用同步配置
    main_config_cmd = os.path.join(config_file_dir, "mid360_realsense_sync.yaml")
    camera_config_cmd = os.path.join(config_file_dir, "camera_realsense_d435i.yaml")

    # Param use_rviz
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Whether to launch Rviz2",
    )

    main_config_arg = DeclareLaunchArgument(
        'main_params_file',
        default_value=main_config_cmd,
        description='Full path to the ROS2 parameters file to use for fast_livo2 nodes',
    )

    camera_config_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value=camera_config_cmd,
        description='Full path to the ROS2 parameters file to use for camera nodes',
    )

    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn', 
        default_value='True',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    main_params_file = LaunchConfiguration('main_params_file')
    camera_params_file = LaunchConfiguration('camera_params_file')
    use_respawn = LaunchConfiguration('use_respawn')

    return LaunchDescription([
        use_rviz_arg,
        main_config_arg,
        camera_config_arg,
        use_respawn_arg,

        # use parameter_blackboard as global parameters server and load camera params
        Node(
            package='demo_nodes_cpp',
            executable='parameter_blackboard',
            name='parameter_blackboard',
            parameters=[
                camera_params_file,
            ],
            output='screen'
        ),
        
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[
                main_params_file,
            ],
            output="screen",
            respawn=use_respawn,
        ),

        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen"
        ),
    ])