# 一键启动脚本： 需要修改成你自己的路径
    ./one_click_launch.sh
# 一键关闭：
    ./one_click_launch.sh stop 

colcon build --packages-select fast_livo   #单独编译fast-livo2
```bash
一、 激光雷达——mid360启动
    
     首先要配置电脑网口ip，打开有线设置，IPv4设置为 “手动” ，IP和子网掩码分别设为：
<!-- 电脑ip需设置为：    "point_data_ip": "192.168.2.50",  255.255.255.0
      雷达ip已预设为：         "ip" : "192.168.2.202", -->
      
```bash
# rviz不启动(建图是要选这种启动方式)
  source install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360_launch.py
  # source install/setup.bash && nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py > lidar.log & #后台启动
  # rviz启动（指定了点云格式为pcl，建图时不使用） 
  source install/setup.bash && ros2 launch livox_ros_driver2 rviz_MID360_launch.py


二、 启动 fast-livo2建图
[camera_init坐标系与重力对齐，配置yaml文件中的： uav： “gravity_align_en: true”]

  source install/setup.bash && ros2 launch fast_livo mapping_mid360_realsense2.launch.py
  source install/setup.bash && nohup ros2 launch fast_livo mapping_mid360_realsense2.launch.py > livox2.log &   #后台启动
```

# 录制rosbag:
```bash
  source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 bag record -a
# 播放rosbag:
  ros2 bag play <bag_name> --loop
# 修复rosbag
  ros2 bag reindex <bag_name>
## 查看时间戳是否对齐：
  ros2 run rqt_bag rqt_bag 
```

# 查看内存占用
```bash
# %MEM ：进程使用的物理内存百分比,建图是，内存涨到50%则会卡顿，大约10min左右
  top 命令查看
# 安装缺失的image_transport插件(解决建图节点挂掉):
sudo apt update && sudo apt install ros-humble-image-transport-plugins

```


<!-- # 时间同步
source install/setup.bash 
ros2 launch fast_livo mapping_mid360_realsense_sync.launch.py

# 时间同步_rviz启动
source install/setup.bash 
ros2 launch livox_ros_driver2 complete_mapping_sync_launch.py


ros2 interface show nav_msgs/msg/Odometry
ros2 topic echo /aft_mapped_to_init --once 

# 时间同步
  source install/setup.bash
  ros2 launch livox_ros_driver2 msg_MID360_sync_launch.py-->

  <!-- # 高程图
```bash
  source install/setup.bash 
# ros2 run elevation_map_node elevation_map_generator
  source install/setup.bash && python3 src/elevation_map_node/src/elevation_map_generator4.py
``` -->



