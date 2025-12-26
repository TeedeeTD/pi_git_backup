#!/bin/bash
# Universal Full CPU Audit v2.2 - ALWAYS SHOW TOP 10
# Compatible: Raspberry Pi (CM4), Ubuntu, Debian, CentOS

# Colors
RED='\033[1;31m'
GREEN='\033[1;32m'
YELLOW='\033[1;33m'
BLUE='\033[1;34m'
CYAN='\033[1;36m'
BOLD='\033[1m'
NC='\033[0m'

# --- FUNCTION: AUTO-BLAME (TOP 10) ---
detect_culprits() {
    echo -e "\n${CYAN}>>> 🕵️  PROCESS AUDIT: DETECTING TOP 10 OFFENDERS...${NC}"
    
    # 1. Top 10 CPU Consumers (User + System)
    echo -e "${BOLD}   [Top 10 CPU Consumers (General Load)]:${NC}"
    ps -eo pid,ppid,%cpu,%mem,comm --sort=-%cpu | head -n 11 | \
    awk 'NR>1 {printf "    🔴 PID: %-6s | CPU: %-6s | RAM: %-5s | CMD: %s\n", $1, $3"%", $4"%", $5}'
    
    # 2. Top 10 System Time Consumers (Kernel Load)
    if command -v pidstat &> /dev/null; then
        echo -e "\n${BOLD}   [Top 10 System Time Consumers (Kernel/Driver Overhead)]:${NC}"
        pidstat -u 1 1 | sed '1,3d' | sort -rn -k 5 | head -n 10 | \
        awk '{printf "    🟠 PID: %-6s | %%usr: %-5s | %%system: %-5s | CMD: %s\n", $3, $4, $5, $NF}'
    else
        echo "    (pidstat not found - install sysstat to see Kernel metrics)"
    fi
}

# Check dependencies
if ! command -v mpstat &> /dev/null; then
    echo -e "${RED}Error: sysstat is required.${NC} Run: sudo apt install sysstat"
    exit 1
fi

echo -e "${BLUE}======================================================${NC}"
echo -e "${BOLD}   FULL CPU AUDIT REPORT (v2.2 - ALWAYS SHOW)${NC}"
echo -e "${BLUE}======================================================${NC}"
echo "Host: $(hostname) | Kernel: $(uname -r)"
date
echo ""

TRIGGER_BLAME=0

# ==============================================================================
# [U] UTILIZATION
# ==============================================================================
echo -e "${BLUE}[U] UTILIZATION (Detailed Load Analysis)${NC}"

mpstat 1 1 | awk '/Average:/ && $2=="all" {
    printf "  User Time (App):      %.1f%%\n", $3
    printf "  System Time (Kernel): %.1f%%\n", $5
    printf "  I/O Wait (Disk):      %.1f%%\n", $6
    printf "  SoftIRQ (Net/HW):     %.1f%%\n", $8
    printf "  Idle:                 %.1f%%\n", $12
}'

# Check High System Time (Threshold > 30%)
SYS_LOAD=$(mpstat 1 1 | awk '/Average:/ && $2=="all" {print $5}' | cut -d. -f1)
if [ "$SYS_LOAD" -gt 30 ]; then
    echo -e "  -> ${RED}CRITICAL: System Time is too high ($SYS_LOAD%)!${NC}"
    TRIGGER_BLAME=1
fi

# ==============================================================================
# [S] SATURATION
# ==============================================================================
echo -e "\n${BLUE}[S] SATURATION (Latency & Congestion)${NC}"

# 1. PSI
if [ -f /proc/pressure/cpu ]; then
    PSI_10=$(grep "some" /proc/pressure/cpu | awk '{print $2}' | cut -d'=' -f2)
    echo -n "  PSI (CPU Pressure):   $PSI_10% "
    if (( $(echo "$PSI_10 > 20.0" | bc -l 2>/dev/null) )); then
        echo -e "${RED}[FAIL] System is lagging.${NC}"
        TRIGGER_BLAME=1
    else
        echo -e "${GREEN}[OK]${NC}"
    fi
else
    echo "  (PSI not supported)"
fi

# 2. Runqueue
VMSTAT=$(vmstat 1 2 | tail -1)
RUN_Q=$(echo $VMSTAT | awk '{print $1}')
CORES=$(nproc)

echo -n "  Run Queue (r):        $RUN_Q (Processes waiting) "
if [ "$RUN_Q" -gt "$CORES" ]; then
    echo -e "${RED}[FAIL] > Cores ($CORES).${NC}"
    TRIGGER_BLAME=1
else
    echo -e "${GREEN}[OK]${NC}"
fi

# ==============================================================================
# [E] ERRORS & PRECURSORS
# ==============================================================================
echo -e "\n${BLUE}[E] ERRORS & PRECURSORS${NC}"

# 1. Temperature
TEMP=""
if command -v vcgencmd &> /dev/null; then
    TEMP=$(vcgencmd measure_temp | egrep -o '[0-9]*\.[0-9]*')
elif command -v sensors &> /dev/null; then
    TEMP=$(sensors | grep '°C' | head -1 | egrep -o '[0-9]+\.[0-9]+' | head -1)
fi

if [ -n "$TEMP" ]; then
    echo -n "  Temperature:          $TEMP°C "
    if (( $(echo "$TEMP > 75.0" | bc -l 2>/dev/null) )); then
        echo -e "${RED}[HIGH]${NC}"
        TRIGGER_BLAME=1
    else
        echo -e "${GREEN}[OK]${NC}"
    fi
fi

# 2. Throttling
if command -v vcgencmd &> /dev/null; then
    CODE=$(vcgencmd get_throttled | cut -d= -f2)
    if [ "$CODE" != "0x0" ]; then
        echo -e "  Throttling Status:    ${RED}FAIL (Pi Code: $CODE)${NC}"
        TRIGGER_BLAME=1
    else
        echo -e "  Throttling Status:    ${GREEN}OK${NC}"
    fi
fi

# ==============================================================================
# EXECUTE AUTO-BLAME (ALWAYS RUN)
# ==============================================================================

# Thông báo trạng thái tổng quan
if [ "$TRIGGER_BLAME" -eq 0 ]; then
    echo -e "\n${GREEN}>>> System Checks Passed (Healthy).${NC}"
    echo -e "${GREEN}>>> Showing Top 10 Processes below as requested:${NC}"
else
    echo -e "\n${RED}>>> System Issues Detected! Analyzing culprits below:${NC}"
fi

# Luôn chạy hàm này
detect_culprits

echo -e "\n${BLUE}===================== END AUDIT =====================${NC}"
