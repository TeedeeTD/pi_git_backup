#!/bin/bash
# ULTIMATE SYSTEM AUDIT v4.0
# Combined: USE Method (Health) + Hardware Benchmark + Troubleshooting Guide

# --- COLORS ---
RED='\033[1;31m'
GREEN='\033[1;32m'
YELLOW='\033[1;33m'
BLUE='\033[1;34m'
CYAN='\033[1;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

echo -e "${BLUE}======================================================${NC}"
echo -e "${BOLD}   ULTIMATE SYSTEM AUDIT (Real-time & Health)${NC}"
echo -e "${BLUE}======================================================${NC}"
echo "Host: $(hostname) | Kernel: $(uname -r)"
date

# Warning flag variable
WARNING_COUNT=0

# Array to store troubleshooting solutions
declare -a SOLUTIONS

add_solution() {
    SOLUTIONS+=("$1")
    ((WARNING_COUNT++))
}

# ==============================================================================
# SECTION 1: HARDWARE & CONFIGURATION CHECK (HARDWARE POTENTIALS)
[cite_start]# [cite: 385, 435, 436]
# ==============================================================================
echo -e "\n${CYAN}[1] HARDWARE & CONFIGURATION CHECK${NC}"

# [cite_start]1.2 CPU Governor Check [cite: 75, 76, 388]
GOVERNOR=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)
if [ "$GOVERNOR" == "performance" ]; then
    echo -e "  CPU Governor:         ${GREEN}OK ($GOVERNOR)${NC}"
else
    echo -e "  CPU Governor:         ${YELLOW}WARNING ($GOVERNOR)${NC}"
    add_solution "Governor Error: CPU is running in power-saving mode. Run: echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor"
fi

# [cite_start]1.3 RAM Benchmark [cite: 112, 389]
echo -n "  RAM Write Speed:      "
RAM_SPEED=$(sysbench memory --memory-block-size=1M --memory-total-size=1G --memory-oper=write run 2>/dev/null | grep "transferred" | awk -F'(' '{print $2}' | cut -d ' ' -f1)
if (( $(echo "$RAM_SPEED > 3000" | bc -l) )); then
    echo -e "${GREEN}$RAM_SPEED MB/s${NC}"
else
    echo -e "${YELLOW}$RAM_SPEED MB/s${NC} (Slow)"
    add_solution "Slow RAM Error: RAM is underperforming (<3000MB/s). Check for excessive background applications."
fi

# [cite_start]1.4 Benchmark Disk Latency (5s) [cite: 117, 390, 417]
echo -n "  Disk Write Latency:   "
DISK_LAT=$(fio --name=test --ioengine=libaio --rw=randwrite --bs=4k --size=64M --numjobs=1 --iodepth=1 --runtime=5 --time_based --end_fsync=1 --minimal 2>/dev/null | awk -F';' '{print $80/1000}')
if (( $(echo "$DISK_LAT < 10.0" | bc -l) )); then
    echo -e "${GREEN}$DISK_LAT ms${NC}"
else
    echo -e "${RED}$DISK_LAT ms${NC} (CRITICAL)"
    add_solution "Disk Latency Error ($DISK_LAT ms): SD card is too slow for real-time logging. Replace with an A2 standard card or switch logging to RAM Disk (/dev/shm)."
fi

# ==============================================================================
# [cite_start]SECTION 2: USE METHOD ANALYSIS (CURRENT HEALTH) [cite: 340, 495]
# ==============================================================================
echo -e "\n${CYAN}[2] USE METHOD ANALYSIS (CURRENT HEALTH)${NC}"

# [cite_start]2.1 [U] Utilization (%System) [cite: 356, 392, 501]
SYS_LOAD=$(mpstat 1 1 | awk '/Average:/ && $2=="all" {print $5}')
echo -n "  Kernel System Load:   "
if (( $(echo "$SYS_LOAD > 20.0" | bc -l) )); then
    echo -e "${RED}$SYS_LOAD%${NC} (High)"
    add_solution "High System Load Error ($SYS_LOAD%): Kernel is overloaded. Use 'pidstat -u 1 1' to find processes spamming System Calls."
else
    echo -e "${GREEN}$SYS_LOAD%${NC}"
fi

# [cite_start]2.2 [S] Saturation (PSI) [cite: 359, 394, 504]
echo -n "  PSI (Lag Index):      "
if [ -f /proc/pressure/cpu ]; then
    PSI_10=$(grep "some" /proc/pressure/cpu | awk '{print $2}' | cut -d'=' -f2)
    if (( $(echo "$PSI_10 > 15.0" | bc -l) )); then
        echo -e "${RED}$PSI_10%${NC} (Lagging)"
        add_solution "CPU Pressure Error ($PSI_10%): System is experiencing CPU bottlenecks. Some tasks are waiting too long."
    else
        echo -e "${GREEN}$PSI_10%${NC}"
    fi
else
    echo "N/A"
fi

# [cite_start]2.3 [E] Errors (Temp & Power) [cite: 104, 309, 362, 396]
# Check Throttling
CODE=$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)
if [ "$CODE" != "0x0" ] && [ -n "$CODE" ]; then
    echo -e "  Power/Throttling:     ${RED}FAIL (Code: $CODE)${NC}"
    add_solution "Power/Throttling Error (Code $CODE): System experienced undervoltage or overheating. Please replace with >3A power supply and check cooling."
else
    echo -e "  Power/Throttling:     ${GREEN}OK${NC}"
fi

# Check Temp
TEMP=$(vcgencmd measure_temp 2>/dev/null | egrep -o '[0-9]*\.[0-9]*')
if [ -z "$TEMP" ]; then TEMP=$(sensors 2>/dev/null | grep '°C' | head -1 | egrep -o '[0-9]+\.[0-9]+' | head -1); fi
echo -n "  Temperature:          "
if (( $(echo "$TEMP > 75.0" | bc -l 2>/dev/null) )); then
    echo -e "${RED}$TEMP°C${NC} (Hot)"
    add_solution "Overheating Error ($TEMP°C): Temperature exceeded safe limit (75°C). Install a cooling fan immediately."
else
    echo -e "${GREEN}$TEMP°C${NC}"
fi

# ==============================================================================
# [cite_start]SECTION 3: PROCESS AUDIT (ALWAYS VISIBLE) [cite: 341, 401]
# ==============================================================================
echo -e "\n${CYAN}[3] PROCESS PROFILING (TOP OFFENDERS)${NC}"

echo -e "${BOLD}   [Top 5 CPU Consumers]:${NC}"
ps -eo pid,ppid,%cpu,%mem,comm --sort=-%cpu | head -n 6 | awk 'NR>1 {printf "    🔴 PID: %-6s | CPU: %-6s | CMD: %s\n", $1, $3"%", $5}'

if command -v pidstat &> /dev/null; then
    echo -e "${BOLD}   [Top 5 System Time Consumers]:${NC}"
    pidstat -u 1 1 | sed '1,3d' | sort -rn -k 5 | head -n 5 | awk '{printf "    🟠 PID: %-6s | Sys: %-5s | CMD: %s\n", $3, $5"%", $NF}'
fi

# ==============================================================================
# SECTION 4: CONCLUSION & TROUBLESHOOTING
# ==============================================================================
echo -e "${BLUE}======================================================${NC}"
if [ "$WARNING_COUNT" -eq 0 ]; then
    echo -e "${GREEN}>>> CONCLUSION: SYSTEM HEALTHY - READY TO FLY.${NC}"
else
    echo -e "${RED}>>> CONCLUSION: DETECTED $WARNING_COUNT ISSUES!${NC}"
    echo -e "${YELLOW}--- TROUBLESHOOTING GUIDE ---${NC}"
    for i in "${!SOLUTIONS[@]}"; do
        echo -e "${RED}[!]${NC} ${SOLUTIONS[$i]}"
    done
fi
echo -e "${BLUE}======================================================${NC}"
