#!/bin/bash

# 简化版一键启动脚本
# 使用 gnome-terminal 分别启动各个节点

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 工作空间路径
FAST_LIVO_WS="/home/nuc11/fast_livo2_bxi4"
ELEVATION_WS="/home/nuc11/elevation_mapping_bxi4"

echo -e "${BLUE}=== 简化版一键启动脚本 ===${NC}"

# 1. 启动激光雷达驱动
echo -e "${GREEN}启动 Livox MID360 激光雷达驱动...${NC}"
gnome-terminal --title="Livox Driver" -- bash -c "cd $FAST_LIVO_WS && source install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360_launch.py; exec bash"

sleep 3

# 2. 启动Fast-LIVO2建图
echo -e "${GREEN}启动 Fast-LIVO2 建图节点...${NC}"
gnome-terminal --title="Fast-LIVO2 Mapping" -- bash -c "cd $FAST_LIVO_WS && source install/setup.bash && ros2 launch fast_livo mapping_mid360_realsense2.launch.py; exec bash"

sleep 3

# 3. 启动elevation mapping节点 (如果存在)
if [ -d "$ELEVATION_WS" ]; then
    echo -e "${GREEN}启动 Elevation Mapping 节点...${NC}"
    
    gnome-terminal --title="Global Elevation" -- bash -c "cd $ELEVATION_WS && source install/setup.bash && python3 global_elevation_map_extractor_launch.py; exec bash"
    
    sleep 2
    
    gnome-terminal --title="Local Elevation" -- bash -c "cd $ELEVATION_WS && source install/setup.bash && python3 local_elevation_map_extractor_z_up_launch.py; exec bash"
    
    sleep 2
    
    gnome-terminal --title="Robot Height" -- bash -c "cd $ELEVATION_WS && source install/setup.bash && python3 simple_robot_height_map_publisher.py; exec bash"
else
    echo -e "${YELLOW}警告: Elevation Mapping 工作空间不存在，跳过相关节点${NC}"
fi

echo -e "${GREEN}=== 所有节点启动完成 ===${NC}"
echo -e "${BLUE}所有节点已在独立终端中启动${NC}"
echo -e "${BLUE}可以直接关闭相应终端来停止对应节点${NC}"
