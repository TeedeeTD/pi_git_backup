import sys
import time
import threading
import socket
import struct
import cv2
import cv2.aruco as aruco
import subprocess
import collections
import csv
from datetime import datetime
import queue
import numpy as np # Cần cài thêm numpy: pip install numpy

# --- CẤU HÌNH HỆ THỐNG ---
RTSP_PUSH_URL = "rtsp://localhost:8554/siyi_aruco"
CAMERA_URL = "rtsp://127.0.0.1:8554/my_camera" 

GPS_PORT = 5555
STREAM_W, STREAM_H = 1280, 720 
FIXED_FPS = 20   # Pi CM4 có thể chịu được 30fps nếu tản nhiệt tốt
FRAME_TIME_MS = 1.0 / FIXED_FPS 

# --- CẤU HÌNH TRACKING (GIMBAL PID) ---
SIYI_IP = "192.168.168.14"  # Đảm bảo đúng IP của SIYI
SIYI_PORT = 37260
TRACKING_ACTIVE = True      # True = Gimbal tự xoay theo ArUco

# Hệ số PID (Cần tinh chỉnh thực tế trên Pi vì độ trễ khác PC)
KP_YAW   = 0.15   
KP_PITCH = 0.15   
DEADZONE = 15     # Tăng deadzone lên chút để gimbal đỡ rung lắc trên Pi

# --- CẤU HÌNH CAMERA MATRIX (CALIBRATION) ---
CAMERA_MATRIX = np.array([
    [717.14, 0.00, 664.29],
    [0.00, 717.88, 354.24],
    [0.00, 0.00, 1.00]
], dtype=float)

DIST_COEFFS = np.array([
    [-0.0764, 0.0742, -0.0013, 0.0019, -0.0176]
])

MARKER_SIZE = 0.142  # Mét

# --- FFMPEG OUTPUT COMMAND (PI HARDWARE V4L2M2M) ---
def get_ffmpeg_command(width, height, fps):
    return [
        'ffmpeg', '-y', 
        '-f', 'rawvideo', '-vcodec', 'rawvideo', '-pix_fmt', 'bgr24', 
        '-s', f'{width}x{height}', '-r', str(fps),
        '-i', '-',
        # --- HARDWARE ENCODER CHO PI ---
        '-c:v', 'h264_v4l2m2m', 
        '-b:v', '2500k', 
        '-pix_fmt', 'yuv420p', 
        '-g', '30',            
        '-bufsize', '1000k',   
        '-f', 'rtsp', RTSP_PUSH_URL
    ]

# --- CLASS 1: SIYI GIMBAL DRIVER (Mang từ PC sang) ---
class SiyiGimbal:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.last_sent = 0

    def _append_crc16(self, data):
        crc = 0
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000: crc = (crc << 1) ^ 0x1021
                else: crc = crc << 1
                crc &= 0xFFFF
        return data + struct.pack('<H', crc)

    def rotate(self, yaw_speed, pitch_speed):
        # Giới hạn tần suất gửi lệnh (Max 20Hz) để tránh làm Pi bị nghẽn mạng
        if time.time() - self.last_sent < 0.05: 
            return
        self.last_sent = time.time()

        yaw_speed = max(min(int(yaw_speed), 100), -100)
        pitch_speed = max(min(int(pitch_speed), 100), -100)
        
        self.seq += 1
        payload = struct.pack('<bb', yaw_speed, pitch_speed) 
        
        # Header: STX(2) | CTRL(1) | LEN(2) | SEQ(2) | CMD(1)
        header = b'\x55\x66' 
        ctrl = b'\x01'       
        length = struct.pack('<H', len(payload))
        seq_bytes = struct.pack('<H', self.seq)
        cmd_id = b'\x07'     # 0x07: Rotate Speed
        
        msg = header + ctrl + length + seq_bytes + cmd_id + payload
        msg_with_crc = self._append_crc16(msg)
        
        try: self.sock.sendto(msg_with_crc, (self.ip, self.port))
        except: pass

    def center(self):
        """Lệnh reset về vị trí trung tâm (Forward)"""
        self.seq += 1
        cmd_id = b'\x08'     # 0x08: Center
        payload = b'\x01'
        
        header = b'\x55\x66'
        ctrl = b'\x01'
        length = struct.pack('<H', len(payload))
        seq_bytes = struct.pack('<H', self.seq)
        
        msg = header + ctrl + length + seq_bytes + cmd_id + payload
        msg_with_crc = self._append_crc16(msg)
        try: self.sock.sendto(msg_with_crc, (self.ip, self.port))
        except: pass

    def stop(self):
        self.rotate(0, 0)

# --- CLASS 2: BUFFERLESS CAMERA READER (Của Pi - Giữ nguyên) ---
class CameraStream:
    def __init__(self, src):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened(): print("❌ Camera Error!")
        else: print("✅ Camera Input: ACTIVE")

        self.ret = False
        self.frame = None
        self.running = True
        self.lock = threading.Lock()
        self.t = threading.Thread(target=self.update, args=(), daemon=True)
        self.t.start()

    def update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.ret = ret
                    self.frame = frame
            else: time.sleep(0.01)

    def read(self):
        with self.lock:
            return self.ret, self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        self.t.join()
        self.cap.release()

# --- CLASS 3: STREAM PUSHER (Của Pi - Giữ nguyên) ---
class StreamPusher:
    def __init__(self, cmd, w, h):
        self.process = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        self.queue = queue.Queue(maxsize=2)
        self.running = True
        self.w, self.h = w, h
        self.t = threading.Thread(target=self.worker, args=(), daemon=True)
        self.t.start()

    def worker(self):
        while self.running:
            try:
                frame = self.queue.get(timeout=1)
                h, w = frame.shape[:2]
                if w != self.w or h != self.h:
                    frame = cv2.resize(frame, (self.w, self.h))
                self.process.stdin.write(frame.tobytes())
                self.queue.task_done()
            except queue.Empty: continue
            except: break

    def write(self, frame):
        if not self.running: return
        try: self.queue.put(frame, block=False)
        except queue.Full: pass 

    def stop(self):
        self.running = False
        self.t.join()
        self.process.stdin.close()
        self.process.wait()

# --- GPS THREAD ---
running_global = True 
current_lat, current_lon = 0.0, 0.0
def gps_thread():
    global current_lat, current_lon
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try: sock.bind(("0.0.0.0", GPS_PORT)); sock.settimeout(1.0)
    except: return
    while running_global:
        try:
            data, _ = sock.recvfrom(1024)
            _, _, lat, lon, _ = struct.unpack('qi3d', data)
            current_lat, current_lon = lat, lon
        except: pass

# --- MAIN ---
def main():
    global running_global
    print(f">>> PI CM4 TRACKING SYSTEM V1.0 (Hardware Enc + PID)...")
    
    # 1. Khởi tạo
    cam = CameraStream(CAMERA_URL)
    time.sleep(2.0)
    if not cam.ret: print("❌ Camera not ready!"); cam.stop(); return
    
    ffmpeg_cmd = get_ffmpeg_command(STREAM_W, STREAM_H, FIXED_FPS)
    pusher = StreamPusher(ffmpeg_cmd, STREAM_W, STREAM_H)
    
    gimbal = SiyiGimbal(SIYI_IP, SIYI_PORT)
    gimbal.center() # Reset gimbal về giữa khi khởi động
    print(f">>> Gimbal Connected: {SIYI_IP}")

    try:
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        parameters = aruco.DetectorParameters_create()
    except:
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
        parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters) if hasattr(aruco, "ArucoDetector") else None

    # CSV Logging
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_file = open(f"pi_track_log_{timestamp_str}.csv", mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Timestamp", "Algo_ms", "FPS", "Dist_m", "ErrX", "ErrY", "Yaw_Cmd", "Pitch_Cmd"])

    frame_count = 0
    start_time_fps = time.perf_counter()
    fps_val = 0
    center_screen = (STREAM_W // 2, STREAM_H // 2)
    
    print(">>> SYSTEM LIVE! TRACKING IS " + ("ON" if TRACKING_ACTIVE else "OFF"))

    while True:
        loop_start = time.perf_counter()

        # B1: Lấy ảnh
        ret, frame = cam.read()
        if not ret or frame is None:
            time.sleep(0.001); continue

        # B2: Xử lý Tracking
        t_algo_start = time.perf_counter()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if detector: corners, ids, _ = detector.detectMarkers(gray)
        else: corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        is_tracking = False
        err_x, err_y = 0, 0
        yaw_cmd, pitch_cmd = 0, 0
        dist_val = 0.0

        if ids is not None and len(ids) > 0:
            is_tracking = True
            
            # Tính Pose 3D (Z distance)
            try:
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS)
                dist_val = tvecs[0][0][2] # Lấy khoảng cách Z của marker đầu tiên
                aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, rvecs[0], tvecs[0], 0.05)
            except: pass

            # Tính toán lỗi PID
            c = corners[0][0]
            cx = int((c[0][0] + c[2][0]) / 2)
            cy = int((c[0][1] + c[2][1]) / 2)

            err_x = cx - center_screen[0]
            err_y = cy - center_screen[1]

            if abs(err_x) > DEADZONE: yaw_cmd = int(err_x * KP_YAW)
            if abs(err_y) > DEADZONE: pitch_cmd = int(-err_y * KP_PITCH)
            
            # Vẽ target line
            cv2.line(frame, center_screen, (cx, cy), (0, 0, 255), 2)

        # Gửi lệnh Gimbal
        if TRACKING_ACTIVE:
            if is_tracking: gimbal.rotate(yaw_cmd, pitch_cmd)
            else: gimbal.rotate(0, 0)

        # Vẽ OSD
        t_algo_end = time.perf_counter()
        algo_ms = (t_algo_end - t_algo_start) * 1000
        
        cv2.drawMarker(frame, center_screen, (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
        cv2.putText(frame, f"FPS: {int(fps_val)} | Algo: {algo_ms:.1f}ms", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        if is_tracking:
            cv2.putText(frame, f"DIST: {dist_val:.2f}m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"CMD: Y{yaw_cmd} P{pitch_cmd}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # B3: Đẩy Stream
        pusher.write(frame)

        # B4: Loop Timing
        elapsed = time.perf_counter() - loop_start
        wait = FRAME_TIME_MS - elapsed
        if wait > 0: time.sleep(wait)
        
        frame_count += 1
        if frame_count >= 15:
            fps_val = frame_count / (time.perf_counter() - start_time_fps)
            start_time_fps = time.perf_counter(); frame_count = 0
            
            # Log CSV (Log chậm thôi để đỡ tốn IO)
            try:
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                csv_writer.writerow([ts, f"{algo_ms:.1f}", f"{fps_val:.1f}", f"{dist_val:.2f}", err_x, err_y, yaw_cmd, pitch_cmd])
            except: pass

    running_global = False
    cam.stop()
    pusher.stop()
    gimbal.stop()
    csv_file.close()

if __name__ == "__main__":
    t = threading.Thread(target=gps_thread, daemon=True)
    t.start()
    main()
