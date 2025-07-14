import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

################### user configure parameters for ros2 start ###################
xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 15.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]

def generate_launch_description():
    # Find config file paths
    config_file_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "fast_livo2.rviz")
    camera_config_cmd = os.path.join(config_file_dir, "camera_realsense_d435i.yaml")
    avia_config_cmd = os.path.join(config_file_dir, "mid360_realsense_sync.yaml")
    
    # Param use_rviz
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Whether to launch Rviz2",
    )

    # Parameter blackboard for camera params (similar to fast_livo)
    parameter_blackboard = Node(
        package='demo_nodes_cpp',
        executable='parameter_blackboard',
        name='parameter_blackboard',
        parameters=[camera_config_cmd],
        output='screen'
    )

    # Livox LiDAR driver
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )

    # RealSense camera
    realsense2_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'rgb_camera.profile': '1280x720x15',
            # 'depth_module.profile': '1280x720x30',
            'pointcloud.enable': True,
            'pointcloud.stream_filter': 2,
            'pointcloud.stream_index_filter': 0,
            'enable_infra1': False,
            'enable_infra2': False,
            'align_depth.enable': True,
            'spatial_filter.enable': True,
            'temporal_filter.enable': True,
            'disparity_filter.enable': True,
            'decimation_filter.enable': True,
            'threshold_filter.enable': False,
            'hole_filling_filter.enable': False,
            'colorizer.enable': False,
            'sync_mode': True,
        }]
    )

    # Sync node
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
            'publish_rate': 15.0
        }]
    )

    # FAST-LIVO2 mapping (using synced data)
    fast_livo_node = Node(
        package='fast_livo',
        executable='fastlivo_mapping',
        name='laserMapping',
        output='screen',
        parameters=[avia_config_cmd]
    )

    # RVIZ2 for visualization
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen"
    )

    return LaunchDescription([
        use_rviz_arg,
        parameter_blackboard,
        livox_driver,
        realsense2_camera_node,
        sync_node,
        fast_livo_node,
        rviz_node
    ]) 