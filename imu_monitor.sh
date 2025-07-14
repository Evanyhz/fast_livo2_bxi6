#!/bin/bash

# IMU缓冲区监控和自动重启脚本
# 监控Fast-LIVO2的IMU缓冲区状态，检测到溢出时自动重启

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 配置参数
FAST_LIVO_WS="/home/nuc11/fast_livo2_bxi4"
LOG_DIR="$FAST_LIVO_WS/logs"
MONITOR_LOG="$LOG_DIR/imu_monitor_$(date +%Y%m%d_%H%M%S).log"
CHECK_INTERVAL=5  # 检查间隔（秒）
BUFFER_OVERFLOW_THRESHOLD=3  # 连续溢出次数阈值
RESTART_COOLDOWN=30  # 重启后的冷却时间（秒）

# 创建日志目录
mkdir -p $LOG_DIR

# 计数器
overflow_count=0
last_restart_time=0

echo -e "${BLUE}=== IMU缓冲区监控脚本启动 ===${NC}"
echo -e "${YELLOW}监控日志: $MONITOR_LOG${NC}"
echo -e "${YELLOW}检查间隔: ${CHECK_INTERVAL}秒${NC}"
echo -e "${YELLOW}溢出阈值: ${BUFFER_OVERFLOW_THRESHOLD}次${NC}"

# 记录日志函数
log_message() {
    local message="$1"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "$message"
    echo "[$timestamp] $message" >> "$MONITOR_LOG"
}

# 检查Fast-LIVO2是否在运行
check_fast_livo_running() {
    if tmux has-session -t "fast_livo_mapping" 2>/dev/null; then
        return 0
    else
        return 1
    fi
}

# 检查IMU缓冲区状态
check_imu_buffer() {
    local latest_log_files=($(ls -t $LOG_DIR/fast_livo_mapping_*.log 2>/dev/null | head -3))
    
    if [ ${#latest_log_files[@]} -eq 0 ]; then
        return 1  # 没有日志文件
    fi
    
    # 检查最新日志文件中的IMU缓冲区信息
    local recent_logs=""
    for log_file in "${latest_log_files[@]}"; do
        recent_logs+=$(tail -n 50 "$log_file" 2>/dev/null)
    done
    
    # 检查是否有IMU缓冲区溢出的迹象
    if echo "$recent_logs" | grep -q "IMU.*buffer.*overflow\|IMU.*队列.*满\|IMU.*缓冲区.*溢出"; then
        return 2  # 明确的溢出错误
    fi
    
    # 检查IMU缓冲区大小异常
    local imu_buffer_size=$(echo "$recent_logs" | grep -o "IMU.*buffer.*size[^0-9]*[0-9]\+" | tail -1 | grep -o "[0-9]\+")
    if [ ! -z "$imu_buffer_size" ] && [ "$imu_buffer_size" -gt 200 ]; then
        return 3  # 缓冲区大小异常
    fi
    
    # 检查是否有长时间没有输出
    local last_output_time=$(echo "$recent_logs" | grep -o "\[.*\]" | tail -1 | tr -d '[]')
    if [ ! -z "$last_output_time" ]; then
        local current_time=$(date +%s)
        local last_time=$(date -d "$last_output_time" +%s 2>/dev/null)
        if [ ! -z "$last_time" ] && [ $((current_time - last_time)) -gt 60 ]; then
            return 4  # 长时间无输出
        fi
    fi
    
    return 0  # 正常
}

# 重启Fast-LIVO2
restart_fast_livo() {
    local current_time=$(date +%s)
    
    # 检查冷却时间
    if [ $((current_time - last_restart_time)) -lt $RESTART_COOLDOWN ]; then
        log_message "${YELLOW}重启冷却中，跳过重启${NC}"
        return
    fi
    
    log_message "${RED}检测到IMU缓冲区问题，正在重启Fast-LIVO2...${NC}"
    
    # 停止Fast-LIVO2
    if tmux has-session -t "fast_livo_mapping" 2>/dev/null; then
        tmux kill-session -t "fast_livo_mapping"
        log_message "${YELLOW}已停止Fast-LIVO2会话${NC}"
    fi
    
    # 等待一段时间
    sleep 5
    
    # 重新启动Fast-LIVO2
    local log_file="$LOG_DIR/fast_livo_mapping_$(date +%Y%m%d_%H%M%S).log"
    local command="cd $FAST_LIVO_WS && source install/setup.bash && ros2 launch fast_livo mapping_mid360_realsense2.launch.py 2>&1 | tee $log_file"
    
    tmux new-session -d -s "fast_livo_mapping" -c "$FAST_LIVO_WS"
    tmux send-keys -t "fast_livo_mapping" "$command" Enter
    
    log_message "${GREEN}Fast-LIVO2已重启，新日志文件: $log_file${NC}"
    
    last_restart_time=$current_time
    overflow_count=0
}

# 显示系统状态
show_system_status() {
    local mem_usage=$(free | grep Mem | awk '{printf("%.1f%%", $3/$2 * 100.0)}')
    local cpu_usage=$(top -bn1 | grep "Cpu(s)" | sed "s/.*, *\([0-9.]*\)%* id.*/\1/" | awk '{print 100 - $1"%"}')
    
    echo -e "${BLUE}系统状态: 内存 $mem_usage, CPU $cpu_usage${NC}"
}

# 主监控循环
log_message "${GREEN}IMU缓冲区监控启动${NC}"

while true; do
    # 检查Fast-LIVO2是否在运行
    if ! check_fast_livo_running; then
        log_message "${YELLOW}Fast-LIVO2未运行，跳过检查${NC}"
        overflow_count=0
        sleep $CHECK_INTERVAL
        continue
    fi
    
    # 检查IMU缓冲区状态
    check_imu_buffer
    buffer_status=$?
    
    case $buffer_status in
        0)
            # 正常状态
            if [ $overflow_count -gt 0 ]; then
                log_message "${GREEN}IMU缓冲区状态恢复正常${NC}"
                overflow_count=0
            fi
            ;;
        1)
            log_message "${YELLOW}无法获取IMU缓冲区状态（日志文件不存在）${NC}"
            ;;
        2)
            log_message "${RED}检测到IMU缓冲区溢出！${NC}"
            overflow_count=$((overflow_count + 1))
            ;;
        3)
            log_message "${YELLOW}IMU缓冲区大小异常${NC}"
            overflow_count=$((overflow_count + 1))
            ;;
        4)
            log_message "${YELLOW}Fast-LIVO2长时间无输出${NC}"
            overflow_count=$((overflow_count + 1))
            ;;
    esac
    
    # 检查是否需要重启
    if [ $overflow_count -ge $BUFFER_OVERFLOW_THRESHOLD ]; then
        restart_fast_livo
    fi
    
    # 显示状态（每分钟一次）
    if [ $(($(date +%s) % 60)) -eq 0 ]; then
        show_system_status
    fi
    
    sleep $CHECK_INTERVAL
done
