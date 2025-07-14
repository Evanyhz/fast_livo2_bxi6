#!/bin/bash

# ROS2话题频率监控脚本
# 监控Fast-LIVO2和Elevation Mapping相关话题的频率

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}=== ROS2话题频率监控 ===${NC}"

# 检查ROS2环境
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}错误: ROS2环境未配置${NC}"
    exit 1
fi

# 重要话题列表
declare -a TOPICS=(
    "/livox/lidar"
    "/livox/imu"
    "/aft_mapped_to_init"
    "/cloud_registered"
    "/path"
    "/local_elevation_map_z_up"
    "/robot_height_map"
    "/tf"
    "/tf_static"
)

# 检查话题是否存在
check_topic_exists() {
    local topic=$1
    if ros2 topic list | grep -q "^$topic$"; then
        return 0
    else
        return 1
    fi
}

# 获取话题频率
get_topic_hz() {
    local topic=$1
    local timeout=5
    
    echo -e "${YELLOW}正在检查话题: $topic${NC}"
    
    if ! check_topic_exists "$topic"; then
        echo -e "${RED}  ✗ 话题不存在${NC}"
        return 1
    fi
    
    # 使用timeout限制检查时间
    local hz_result=$(timeout $timeout ros2 topic hz "$topic" 2>/dev/null | grep "average rate" | head -1)
    
    if [ -z "$hz_result" ]; then
        echo -e "${RED}  ✗ 无数据或超时${NC}"
        return 1
    fi
    
    echo -e "${GREEN}  ✓ $hz_result${NC}"
    return 0
}

# 快速检查模式
quick_check() {
    echo -e "${BLUE}=== 快速频率检查 ===${NC}"
    
    for topic in "${TOPICS[@]}"; do
        get_topic_hz "$topic"
    done
}

# 实时监控模式
monitor_mode() {
    echo -e "${BLUE}=== 实时监控模式 (按Ctrl+C退出) ===${NC}"
    
    while true; do
        clear
        echo -e "${BLUE}=== ROS2话题频率监控 - $(date) ===${NC}"
        
        for topic in "${TOPICS[@]}"; do
            get_topic_hz "$topic"
        done
        
        echo -e "\n${YELLOW}5秒后刷新...${NC}"
        sleep 5
    done
}

# 详细模式 - 显示话题详细信息
detailed_mode() {
    echo -e "${BLUE}=== 详细模式 ===${NC}"
    
    for topic in "${TOPICS[@]}"; do
        if check_topic_exists "$topic"; then
            echo -e "${GREEN}话题: $topic${NC}"
            
            # 获取话题类型
            local topic_type=$(ros2 topic info "$topic" | grep "Type:" | awk '{print $2}')
            echo -e "  类型: $topic_type"
            
            # 获取发布者数量
            local publishers=$(ros2 topic info "$topic" | grep -A 10 "Publisher count:" | grep -c "Node name:")
            echo -e "  发布者: $publishers"
            
            # 获取订阅者数量
            local subscribers=$(ros2 topic info "$topic" | grep -A 10 "Subscription count:" | grep -c "Node name:")
            echo -e "  订阅者: $subscribers"
            
            # 获取频率
            echo -e "${YELLOW}  正在测量频率...${NC}"
            local hz_result=$(timeout 3 ros2 topic hz "$topic" 2>/dev/null | grep "average rate" | head -1)
            if [ ! -z "$hz_result" ]; then
                echo -e "  ${GREEN}$hz_result${NC}"
            else
                echo -e "  ${RED}无数据${NC}"
            fi
            
            echo ""
        fi
    done
}

# 自定义话题检查
custom_topic() {
    local topic=$1
    if [ -z "$topic" ]; then
        echo -e "${RED}请提供话题名称${NC}"
        return 1
    fi
    
    echo -e "${BLUE}=== 自定义话题检查: $topic ===${NC}"
    get_topic_hz "$topic"
}

# 显示帮助
show_help() {
    echo -e "${BLUE}使用方法:${NC}"
    echo -e "  $0 [选项] [话题名]"
    echo -e ""
    echo -e "${BLUE}选项:${NC}"
    echo -e "  quick     - 快速检查所有重要话题 (默认)"
    echo -e "  monitor   - 实时监控模式"
    echo -e "  detailed  - 详细模式"
    echo -e "  custom    - 检查自定义话题"
    echo -e "  list      - 列出所有活动话题"
    echo -e "  help      - 显示帮助"
    echo -e ""
    echo -e "${BLUE}示例:${NC}"
    echo -e "  $0                           # 快速检查"
    echo -e "  $0 monitor                   # 实时监控"
    echo -e "  $0 custom /livox/lidar       # 检查特定话题"
    echo -e "  $0 list                      # 列出所有话题"
}

# 列出所有活动话题
list_topics() {
    echo -e "${BLUE}=== 所有活动话题 ===${NC}"
    ros2 topic list | sort
}

# 主程序
case "${1:-quick}" in
    "quick")
        quick_check
        ;;
    "monitor")
        monitor_mode
        ;;
    "detailed")
        detailed_mode
        ;;
    "custom")
        custom_topic "$2"
        ;;
    "list")
        list_topics
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
