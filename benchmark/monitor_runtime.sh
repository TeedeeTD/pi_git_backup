#!/bin/bash
clear
while true; do
    TEMP=$(vcgencmd measure_temp)
    CLOCK=$(vcgencmd measure_clock arm | awk -F= '{print $2/1000000 " MHz"}')
    # Tải Core 3 (Quan trọng nhất)
    CORE3=$(mpstat -P 3 1 1 | tail -1 | awk '{print $3 + $4 "%"}')
    # Tải Core 0-2 (Stream)
    OTHERS=$(mpstat -P 0,1,2 1 1 | grep "Average" | tail -1 | awk '{print $3 + $4 "%"}')
    
    echo "=== RUNTIME MONITOR (Ctrl+C to exit) ==="
    echo " Hardware: $TEMP | $CLOCK"
    echo "----------------------------------------"
    echo " CORE 3 (CONTROL): $CORE3  (Nên < 50%)"
    echo " CORE 0-2 (VIDEO): $OTHERS (Có thể cao)"
    echo "----------------------------------------"
    
    tput cuu 5
done
