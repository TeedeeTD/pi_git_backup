#!/bin/bash

# --- CAU HINH ---
WS_DIR=~/ros2_ws
PKG_NAME="advanced_profiling"
NODE_NAME="dds_stress_node"

echo "========================================================"
echo "   KHOI TAO BAI TEST ROS 2 NANG CAO (FIXED 64-BIT)      "
echo "========================================================"

# 1. Kiem tra va cai dat thu vien Python can thiet
echo "[1/4] Kiem tra thu vien phu tro..."
pip3 install setproctitle numpy > /dev/null 2>&1

# 2. Tao Package ROS 2
echo "[2/4] Tao Package '$PKG_NAME'..."
mkdir -p $WS_DIR/src
cd $WS_DIR/src

if [ -d "$PKG_NAME" ]; then
    echo " -> Package da ton tai. Se cap nhat code moi nhat."
else
    ros2 pkg create --build-type ament_python --node-name $NODE_NAME $PKG_NAME > /dev/null
    echo " -> Da tao package moi."
fi

# 3. Viet Code Python (Da fix loi Segfault cho he thong 64-bit)
echo "[3/4] Ghi code Python (Da tich hop Thread Naming)..."
cat > $WS_DIR/src/$PKG_NAME/$PKG_NAME/$NODE_NAME.py << 'EOF'
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import time
import threading
import ctypes
import numpy as np
import setproctitle

# --- CAU HINH DINH DANH LUONG (THREAD NAMING) ---
# Day la phan sua loi Segfault: Khai bao ro kieu du lieu cho Ctypes tren 64-bit
try:
    lib = ctypes.CDLL("libc.so.6")
    # pthread_self() tra ve unsigned long (64-bit)
    lib.pthread_self.restype = ctypes.c_ulong
    # pthread_setname_np() nhan vao (unsigned long, char*)
    lib.pthread_setname_np.argtypes = [ctypes.c_ulong, ctypes.c_char_p]
    lib.pthread_setname_np.restype = ctypes.c_int
except Exception as e:
    lib = None

def set_thread_name(name):
    """Dat ten cho luong de hien thi trong htop"""
    # 1. Dat ten o cap do Python
    threading.current_thread().name = name
    
    # 2. Dat ten o cap do OS (Linux pthread)
    if lib:
        try:
            # Gioi han ten luong la 15 ky tu (Linux Kernel limit)
            name_bytes = name[:15].encode('utf-8')
            lib.pthread_setname_np(lib.pthread_self(), name_bytes)
        except:
            pass

class AdvancedStressNode(Node):
    def __init__(self):
        super().__init__('dds_stress_node')
        # Dat ten Process chinh (nhin thay trong Glances/htop mac dinh)
        setproctitle.setproctitle("ros2_adv_test") 

        # --- TAO TAI ---
        # 1. Publisher: Gui du lieu String lon de gay ap luc len Middleware (DDS)
        # Muc dich: Do luong Serialization Overhead (PDF Section 2.2.1)
        self.pub = self.create_publisher(String, 'heavy_topic', 10)
        self.timer_pub = self.create_timer(0.05, self.publish_job) # 20Hz

        # 2. Subscriber: Nhan va tinh toan nang
        self.sub = self.create_subscription(String, 'heavy_topic', self.sub_callback, 10)
        
        # Tao data gia (~500KB)
        self.dummy_data = "a" * 500000 
        self.get_logger().info("Node Started. Thread naming active.")

    def publish_job(self):
        # Dinh danh luong nay la "ROS_Pub"
        set_thread_name("ROS_Pub")
        
        msg = String()
        # Them time de data thay doi lien tuc
        msg.data = self.dummy_data + str(time.time())
        self.pub.publish(msg)

    def sub_callback(self, msg):
        # Dinh danh luong nay la "ROS_Sub"
        set_thread_name("ROS_Sub")
        
        # Gia lap tinh toan nang (Matrix Multiplication)
        size = 120
        np.dot(np.random.rand(size, size), np.random.rand(size, size))

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedStressNode()
    
    # Dinh danh luong chinh (Main Thread) la "ROS_Exec"
    set_thread_name("ROS_Exec")
    
    # Su dung MultiThreadedExecutor voi 4 luong (tan dung 4 core CM4)
    # PDF Section 2.1.2: Can than voi Overhead
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

# 4. Build Package
echo "[4/4] Build Workspace..."
cd $WS_DIR
colcon build --packages-select $PKG_NAME > /dev/null

echo "========================================================"
echo "   CAI DAT HOAN TAT - SAN SANG CHAY                   "
echo "========================================================"
echo "HUONG DAN DO LUONG (Bat buoc lam theo):"
echo ""
echo "BUOC 1: Chay Node o Terminal nay:"
echo "   source ~/ros2_ws/install/setup.bash"
echo "   ros2 run advanced_profiling dds_stress_node"
echo ""
echo "BUOC 2: Mo Terminal khac va chay 'htop':"
echo "   - An F2 (Setup) -> Display options"
echo "   - Tich vao [x] Show custom thread names"
echo "   - Tich vao [x] Show threads"
echo "   - An F10 (Save)"
echo "   - An F5 (Tree View) va tim 'ros2_adv_test'"
echo ""
