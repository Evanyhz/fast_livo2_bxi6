#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'grid_size',
            default_value='0.1',
            description='Grid cell size in meters'
        ),
        
        DeclareLaunchArgument(
            'map_width',
            default_value='1.0',
            description='Map width in meters (left-right total)'
        ),
        
        DeclareLaunchArgument(
            'map_height', 
            default_value='1.6',
            description='Map height in meters (front-back total)'
        ),
        
        DeclareLaunchArgument(
            'update_rate',
            default_value='10.0',
            description='Map update rate in Hz'
        ),
        
        Node(
            package='elevation_map_node',
            executable='elevation_map_generator',
            name='elevation_map_generator',
            output='screen',
            parameters=[{
                'grid_size': LaunchConfiguration('grid_size'),
                'map_width': LaunchConfiguration('map_width'),
                'map_height': LaunchConfiguration('map_height'),
                'update_rate': LaunchConfiguration('update_rate'),
            }],
            remappings=[
                ('/cloud_registered', '/cloud_registered'),
                ('/aft_mapped_to_init', '/aft_mapped_to_init'),
                ('/elevation_map', '/elevation_map')
            ]
        )
    ]) 