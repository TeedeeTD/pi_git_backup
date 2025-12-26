#!/bin/bash

# ==============================================================================
# SCRIPT: CPU Health Check - USE Method (Utilization, Saturation, Errors)
# MỤC ĐÍCH: Kiểm tra nhanh sức khỏe CPU theo chuẩn công nghiệp
# YÊU CẦU: Gói sysstat (cài bằng: apt-get install sysstat hoặc yum install sysstat)
# ==============================================================================

# Màu sắc cho output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}======================================================${NC}"
echo -e "${CYAN}   CPU HEALTH CHECK REPORT (USE METHOD)${NC}"
echo -e "${CYAN}======================================================${NC}"
date
echo ""

# Kiểm tra công cụ cần thiết
if ! command -v mpstat &> /dev/null; then
    echo -e "${RED}[!] Cảnh báo: 'mpstat' chưa được cài đặt. Vui lòng cài gói 'sysstat' để có kết quả chính xác nhất.${NC}"
    echo -e "    Lệnh cài: sudo apt install sysstat (Ubuntu) hoặc sudo yum install sysstat (CentOS)"
    echo ""
fi

# Lấy số lượng CPU Cores
CORES=$(nproc)
echo -e "CPU Info: ${YELLOW}$CORES Cores${NC}"

# ==============================================================================
# 1. UTILIZATION (Mức độ sử dụng)
# ==============================================================================
echo -e "\n${CYAN}[1] U - UTILIZATION (Mức độ sử dụng)${NC}"
echo "------------------------------------------------------"

# Lấy thông số CPU tổng quan từ top
echo ">>> Tổng quan (User/System/Idle):"
top -bn1 | grep "Cpu(s)" | awk '{print "User: " $2 "%, System: " $4 "%, Idle: " $8 "%"}'

# Check từng Core (nếu có mpstat)
if command -v mpstat &> /dev/null; then
    echo -e "\n>>> Chi tiết từng Core (Tìm kiếm Core bị lệch tải - Single Thread Bottleneck):"
    mpstat -P ALL 1 1 | awk '/Average:/ && $2 != "all" {print "Core " $2 ": User " $3 "% | Sys " $5 "% | Idle " $12 "%"}' | while read line; do
        idle=$(echo $line | awk '{print $NF}' | cut -d'%' -f1 | cut -d'.' -f1)
        if [ "$idle" -lt 5 ]; then
            echo -e "${RED}$line (HOT!)${NC}"
        else
            echo "$line"
        fi
    done
else
    echo "(!) Bỏ qua chi tiết từng core do thiếu mpstat."
fi

# ==============================================================================
# 2. SATURATION (Độ bão hòa - Quan trọng nhất)
# ==============================================================================
echo -e "\n${CYAN}[2] S - SATURATION (Độ bão hòa & Latency)${NC}"
echo "------------------------------------------------------"

# Load Average
LOAD_1MIN=$(uptime | grep -o 'load average.*' | awk -F': ' '{print $2}' | cut -d',' -f1)
echo -e ">>> Load Average (1 min): ${YELLOW}$LOAD_1MIN${NC} (vs $CORES Cores)"

# Logic so sánh Load vs Cores
IS_SATURATED=$(echo "$LOAD_1MIN > $CORES" | bc -l)
if [ "$IS_SATURATED" -eq 1 ]; then
    echo -e "${RED}[!] CẢNH BÁO: Load Average > Số Cores. Hệ thống đang có Latency (Tiến trình phải chờ)!${NC}"
else
    echo -e "${GREEN}[OK] Load Average nằm trong ngưỡng xử lý.${NC}"
fi

# VMSTAT Check (Run Queue, Blocked, Context Switch)
echo -e "\n>>> Run Queue & Context Switches (vmstat):"
echo "r  = Process chờ chạy (Run Queue) -> Nếu cao hơn $CORES là xấu"
echo "b  = Process chờ I/O (Blocked)    -> Nếu cao là do Disk/Network"
echo "cs = Context Switches/sec"
echo "------------------------------------------------------"
vmstat 1 5 | tail -n 5

# ==============================================================================
# 3. ERRORS (Lỗi & Throttling)
# ==============================================================================
echo -e "\n${CYAN}[3] E - ERRORS (Lỗi & Throttling)${NC}"
echo "------------------------------------------------------"

# Check Kernel Logs cho Hardware Throttling
echo ">>> Kiểm tra log hệ thống (dmesg) cho Hardware Throttling:"
if dmesg | grep -i "throttle" | grep -v "checking"; then
    echo -e "${RED}[!] PHÁT HIỆN: Có log về 'throttle'. CPU có thể bị quá nhiệt!${NC}"
else
    echo -e "${GREEN}[OK] Không tìm thấy log Hardware Throttling gần đây.${NC}"
fi

# Check Container Throttling (Cgroup v1)
CGROUP_FILE="/sys/fs/cgroup/cpu/cpu.stat"
if [ -f "$CGROUP_FILE" ]; then
    echo -e "\n>>> Kiểm tra Container Throttling (Nếu chạy trong Docker/K8s):"
    cat $CGROUP_FILE | grep "nr_throttled"
    THROTTLED=$(cat $CGROUP_FILE | grep "nr_throttled" | awk '{print $2}')
    if [ "$THROTTLED" -gt 0 ]; then
        echo -e "${YELLOW}[!] Note: Container này đã bị bóp CPU $THROTTLED lần.${NC}"
    else
        echo -e "${GREEN}[OK] Container chưa bị bóp CPU lần nào.${NC}"
    fi
fi

echo -e "\n${CYAN}=================== KẾT THÚC KIỂM TRA ===================${NC}"
