#!/bin/bash
# MASTER BENCHMARK SUITE v2.0
# Gộp: Hardware Check + USE Method + Network/AI Check

RED='\033[1;31m'
GREEN='\033[1;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# --- CẤU HÌNH IP MÁY NHẬN STREAM (Sửa dòng này) ---
TARGET_IP="192.168.1.10"

echo "=================================================="
echo "      HỆ THỐNG KIỂM TRA TOÀN DIỆN (PRE-FLIGHT)"
echo "=================================================="

# 1. KIỂM TRA ISOLATION (BẮT BUỘC)
ISOLATED=$(cat /sys/devices/system/cpu/isolated 2>/dev/null)
if [[ "$ISOLATED" == *"3"* ]]; then
    echo -e "[1] Core Isolation:   ${GREEN}OK (Core $ISOLATED)${NC}"
else
    echo -e "[1] Core Isolation:   ${RED}FAIL (Chưa set isolcpus=3)${NC}"
fi

# 2. KIỂM TRA NGUỒN & NHIỆT (THROTTLING)
THROTTLED=$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)
TEMP=$(vcgencmd measure_temp 2>/dev/null)
if [ "$THROTTLED" == "0x0" ]; then
    echo -e "[2] Power Supply:     ${GREEN}STABLE (0x0)${NC}"
else
    echo -e "[2] Power Supply:     ${RED}UNSTABLE ($THROTTLED) - Thay nguồn ngay!${NC}"
fi
echo -e "    Temperature:      $TEMP"

# 3. BENCHMARK RAM (Tốc độ copy ảnh)
echo -n "[3] RAM Write Speed:  "
RAM=$(sysbench memory --memory-block-size=1M --memory-total-size=1G --memory-oper=write run 2>/dev/null | grep "transferred" | awk -F'(' '{print $2}' | cut -d ' ' -f1)
if (( $(echo "$RAM > 3000" | bc -l) )); then
    echo -e "${GREEN}$RAM MB/sec${NC}"
else
    echo -e "${YELLOW}$RAM MB/sec (Hơi chậm)${NC}"
fi

# 4. BENCHMARK DISK (Độ trễ ghi log)
echo -n "[4] Disk Latency:     "
DISK=$(fio --name=test --ioengine=libaio --rw=randwrite --bs=4k --size=64M --numjobs=1 --iodepth=1 --runtime=3 --time_based --end_fsync=1 --minimal 2>/dev/null | awk -F';' '{print $80/1000}')
if (( $(echo "$DISK < 10.0" | bc -l) )); then
    echo -e "${GREEN}$DISK ms${NC}"
else
    echo -e "${RED}$DISK ms (QUÁ CHẬM - Đừng ghi log vào thẻ này)${NC}"
fi

# 5. BENCHMARK MẠNG (Ping Flood & Jitter)
echo -n "[5] Network Stability: "
PING_LOSS=$(sudo ping -f -c 500 $TARGET_IP 2>&1 | grep "packet loss" | awk '{print $6}')
if [ "$PING_LOSS" == "0%" ]; then
    echo -e "${GREEN}OK (0% Loss)${NC}"
else
    echo -e "${RED}FAIL ($PING_LOSS Loss)${NC}"
fi

echo -n "    Network Jitter:    "
if command -v iperf3 &> /dev/null; then
    JITTER=$(iperf3 -c $TARGET_IP -u -b 10M -t 5 --json 2>/dev/null | grep "jitter_ms" | head -1 | awk '{print $2}' | tr -d ',')
    if (( $(echo "$JITTER < 2.0" | bc -l 2>/dev/null) )); then
        echo -e "${GREEN}${JITTER}ms${NC}"
    else
        echo -e "${RED}${JITTER}ms${NC} (High Jitter)"
    fi
else
    echo "Skip (Cần cài iperf3)"
fi

# 6. BENCHMARK AI (Simulation)
echo -e "[6] AI Inference Speed (MobileNetV2):"
cat << EOF > ai_test_temp.py
import time
import numpy as np
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
try:
    import tensorflow as tf
    model = tf.keras.applications.MobileNetV2(weights='imagenet', include_top=False, input_shape=(224, 224, 3))
    input_data = np.random.rand(1, 224, 224, 3).astype(np.float32)
    model.predict(input_data, verbose=0) # Warmup
    times = []
    for i in range(20):
        start = time.time()
        model.predict(input_data, verbose=0)
        times.append((time.time() - start) * 1000)
    print(f"{sum(times)/len(times):.2f}")
except:
    print("ERROR")
EOF

if python3 -c "import tensorflow" &> /dev/null; then
    RESULT=$(python3 ai_test_temp.py)
    if [ "$RESULT" == "ERROR" ]; then
        echo -e "    -> ${RED}Lỗi thư viện AI${NC}"
    else
        echo -e "    -> ${GREEN}$RESULT ms/frame${NC}"
    fi
else
    echo -e "    -> ${RED}Chưa cài TensorFlow${NC}"
fi
rm ai_test_temp.py

echo "=================================================="
