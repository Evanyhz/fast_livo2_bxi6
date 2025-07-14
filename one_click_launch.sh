#!/bin/bash

# 一键启动脚本 - 同时启动fast_livo2和elevation_mapping工作空间的所有节点

需要提前安装终端复用器：Tmux
    sudo apt update && sudo apt install tmux -y
    
# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 工作空间路径
FAST_LIVO_WS="/home/nuc11/fast_livo2_bxi4"
ELEVATION_WS="/home/nuc11/elevation_mapping_bxi4"

# 日志文件路径
LOG_DIR="${FAST_LIVO_WS}/logs"
mkdir -p $LOG_DIR

echo -e "${BLUE}=== Fast-LIVO2 + Elevation Mapping 一键启动脚本 ===${NC}"
echo -e "${YELLOW}启动时间: $(date)${NC}"

# 检查工作空间是否存在
check_workspace() {
    local ws_path=$1
    local ws_name=$2
    if [ ! -d "$ws_path" ]; then
        echo -e "${RED}错误: $ws_name 工作空间不存在: $ws_path${NC}"
        return 1
    fi
    if [ ! -f "$ws_path/install/setup.bash" ]; then
        echo -e "${RED}错误: $ws_name 工作空间未编译或setup.bash不存在${NC}"
        return 1
    fi
    return 0
}

# 检查工作空间
echo -e "${BLUE}检查工作空间...${NC}"
check_workspace $FAST_LIVO_WS "Fast-LIVO2" || exit 1
check_workspace $ELEVATION_WS "Elevation Mapping" || exit 1

# 创建会话管理函数
create_session() {
    local session_name=$1
    local command=$2
    local log_file=$3
    
    if tmux has-session -t "$session_name" 2>/dev/null; then
        echo -e "${YELLOW}会话 $session_name 已存在，正在重启...${NC}"
        tmux kill-session -t "$session_name"
    fi
    
    echo -e "${GREEN}启动会话: $session_name${NC}"
    tmux new-session -d -s "$session_name" -c "$PWD"nuc11@nuc11-NUC11TNKi5:~/fast_livo2_bxi4$ ./one_click_launch.sh
=== Fast-LIVO2 + Elevation Mapping 一键启动脚本 ===
启动时间: 2025年 07月 14日 星期一 15:40:33 CST
检查工作空间...
=== 开始启动所有节点 ===
启动 Livox MID360 激光雷达驱动...
启动会话: livox_driver
日志文件: /home/nuc11/fast_livo2_bxi4/logs/livox_driver_20250714_154033.log
等待激光雷达启动...
启动 Fast-LIVO2 建图节点...
启动会话: fast_livo_mapping
日志文件: /home/nuc11/fast_livo2_bxi4/logs/fast_livo_mapping_20250714_154040.log
等待建图节点启动...
启动 Elevation Mapping 节点...
启动 全局高程地图提取器...
启动会话: global_elevation
日志文件: /home/nuc11/fast_livo2_bxi4/logs/global_elevation_20250714_154047.log
启动 局部高程地图提取器...
启动会话: local_elevation
日志文件: /home/nuc11/fast_livo2_bxi4/logs/local_elevation_20250714_154051.log
启动 机器人高度地图发布器...
启动会话: robot_height
日志文件: /home/nuc11/fast_livo2_bxi4/logs/robot_height_20250714_154055.log
=== 所有节点启动完成 ===
日志文件位置: /home/nuc11/fast_livo2_bxi4/logs
使用 'tmux list-sessions' 查看所有会话
使用 './one_click_launch.sh status' 查看节点状态
使用 './one_click_launch.sh monitor' 监控节点状态
nuc11@nuc11-NUC11TNKi5:~/fast_livo2_bxi4$ ros2 topic list 
/LIVO2/imu_propagate
/Laser_map
/aft_mapped_to_init
/camera/camera/color/image_raw
/clicked_point
/cloud_effected
/cloud_registered
/cloud_visual_sub_map_before
/dyn_obj
/dyn_obj_dbg_hist
/dyn_obj_removed
/in/compressed
/initialpose
/livox/imu
/livox/lidar
/local_elevation_map_z_up
/mavros/vision_pose/pose
/motion_commands
/move_base_simple/goal
/parameter_events
/path
/planes
/planner_normal
/planner_normal_array
/rgb_img
/rgb_img/compressed
/rgb_img/compressedDepth
/rgb_img/theora
/robot_height_map
/rosout
/simulation/actuators_cmds
/simulation/imu_data
/simulation/joint_states
/simulation/odom
/tf
/tf_static
/visualization_marker
/visualization_marker_array
/voxels
    tmux send-keys -t "$session_name" "$command" Enter
    
    if [ ! -z "$log_file" ]; then
        echo -e "${BLUE}日志文件: $log_file${NC}"
    fi
}

# 启动节点函数
start_node() {
    local session_name=$1
    local workspace=$2
    local command=$3
    local description=$4
    local log_file="$LOG_DIR/${session_name}_$(date +%Y%m%d_%H%M%S).log"
    
    echo -e "${GREEN}启动 $description...${NC}"
    
    # 构造完整命令
    local full_command="cd $workspace && source install/setup.bash && $command 2>&1 | tee $log_file"
    
    create_session "$session_name" "$full_command" "$log_file"
    sleep 2
}

# 停止所有相关会话
stop_all_sessions() {
    echo -e "${YELLOW}停止所有相关会话...${NC}"
    sessions=("livox_driver" "fast_livo_mapping" "global_elevation" "local_elevation" "robot_height")
    for session in "${sessions[@]}"; do
        if tmux has-session -t "$session" 2>/dev/null; then
            echo -e "${YELLOW}停止会话: $session${NC}"
            tmux kill-session -t "$session"
        fi
    done
}

# 显示帮助信息
show_help() {
    echo -e "${BLUE}使用方法:${NC}"
    echo -e "  $0 [选项]"
    echo -e ""
    echo -e "${BLUE}选项:${NC}"
    echo -e "  start    - 启动所有节点 (默认)"
    echo -e "  stop     - 停止所有节点"
    echo -e "  restart  - 重启所有节点"
    echo -e "  status   - 查看节点状态"
    echo -e "  monitor  - 监控节点状态"
    echo -e "  help     - 显示帮助信息"
    echo -e ""
    echo -e "${BLUE}tmux 会话管理:${NC}"
    echo -e "  tmux list-sessions              - 查看所有会话"
    echo -e "  tmux attach-session -t <name>   - 连接到指定会话"
    echo -e "  tmux kill-session -t <name>     - 停止指定会话"
    echo -e "  Ctrl+B, D                       - 从会话中分离"
}

# 查看节点状态
show_status() {
    echo -e "${BLUE}=== 节点状态 ===${NC}"
    sessions=("livox_driver" "fast_livo_mapping" "global_elevation" "local_elevation" "robot_height")
    for session in "${sessions[@]}"; do
        if tmux has-session -t "$session" 2>/dev/null; then
            echo -e "${GREEN}✓ $session - 运行中${NC}"
        else
            echo -e "${RED}✗ $session - 未运行${NC}"
        fi
    done
    
    echo -e "\n${BLUE}=== ROS2 节点状态 ===${NC}"
    source /opt/ros/humble/setup.bash
    ros2 node list 2>/dev/null | grep -E "(livox|fast_livo|elevation|robot_height)" || echo -e "${YELLOW}没有找到相关ROS2节点${NC}"
}

# 监控节点状态
monitor_nodes() {
    echo -e "${BLUE}=== 节点监控 (按 Ctrl+C 退出) ===${NC}"
    while true; do
        clear
        echo -e "${BLUE}=== 节点监控 - $(date) ===${NC}"
        show_status
        
        echo -e "\n${BLUE}=== 系统资源 ===${NC}"
        echo -e "内存使用率: $(free | grep Mem | awk '{printf("%.1f%%", $3/$2 * 100.0)}')"
        echo -e "CPU使用率: $(top -bn1 | grep "Cpu(s)" | sed "s/.*, *\([0-9.]*\)%* id.*/\1/" | awk '{print 100 - $1"%"}')"
        
        echo -e "\n${YELLOW}5秒后刷新...${NC}"
        sleep 5
    done
}

# 启动所有节点
start_all_nodes() {
    echo -e "${BLUE}=== 开始启动所有节点 ===${NC}"
    
    # 1. 启动激光雷达驱动
    start_node "livox_driver" "$FAST_LIVO_WS" \
        "ros2 launch livox_ros_driver2 msg_MID360_launch.py" \
        "Livox MID360 激光雷达驱动"
    
    # 等待激光雷达启动
    echo -e "${YELLOW}等待激光雷达启动...${NC}"
    sleep 5
    
    # 2. 启动Fast-LIVO2建图
    start_node "fast_livo_mapping" "$FAST_LIVO_WS" \
        "ros2 launch fast_livo mapping_mid360_realsense2.launch.py" \
        "Fast-LIVO2 建图节点"
    
    # 等待建图节点启动
    echo -e "${YELLOW}等待建图节点启动...${NC}"
    sleep 5
    
    # 3. 启动elevation mapping节点
    if [ -d "$ELEVATION_WS" ]; then
        echo -e "${BLUE}启动 Elevation Mapping 节点...${NC}"
        
        start_node "global_elevation" "$ELEVATION_WS" \
            "ros2 launch elevation_mapping global_elevation_map_extractor_launch.py" \
            "全局高程地图提取器"
        
        sleep 2
        
        start_node "local_elevation" "$ELEVATION_WS" \
            "ros2 launch elevation_mapping local_elevation_map_extractor_z_up_launch.py" \
            "局部高程地图提取器"
        
        sleep 2
        
        start_node "robot_height" "$ELEVATION_WS" \
            "python3 simple_robot_height_map_publisher.py" \
            "机器人高度地图发布器"
    else
        echo -e "${YELLOW}警告: Elevation Mapping 工作空间不存在，跳过相关节点${NC}"
    fi
    
    echo -e "${GREEN}=== 所有节点启动完成 ===${NC}"
    echo -e "${BLUE}日志文件位置: $LOG_DIR${NC}"
    echo -e "${BLUE}使用 'tmux list-sessions' 查看所有会话${NC}"
    echo -e "${BLUE}使用 '$0 status' 查看节点状态${NC}"
    echo -e "${BLUE}使用 '$0 monitor' 监控节点状态${NC}"
}

# 主程序
case "${1:-start}" in
    "start")
        start_all_nodes
        ;;
    "stop")
        stop_all_sessions
        ;;
    "restart")
        stop_all_sessions
        sleep 3
        start_all_nodes
        ;;
    "status")
        show_status
        ;;
    "monitor")
        monitor_nodes
        ;;
    "help"|"-h"|"--help")
        show_help
        ;;
    *)
        echo -e "${RED}未知选项: $1${NC}"
        show_help
        exit 1
        ;;
esac
