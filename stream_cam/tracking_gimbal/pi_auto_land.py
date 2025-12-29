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
import numpy as np
import math

# ==========================================
# --- CẤU HÌNH CHIẾN THUẬT HẠ CÁNH ---
# ==========================================
# Độ cao kích hoạt chế độ khóa Gimbal (Mét)
# Khi Z < mức này -> Gimbal sẽ dừng tracking và khóa cứng
LANDING_ALTITUDE = 2.5 

# Hành động khi khóa: 
# True = Về giữa (Nhìn thẳng 0 độ)
# False = Giữ nguyên góc hiện tại (Stop tại chỗ)
AUTO_CENTER_ON_LOCK = True 

# ==========================================
# --- CẤU HÌNH HỆ THỐNG ---
# ==========================================
DETECT_SCALE = 0.7 
SKIP_FRAME = 1      

RTSP_PUSH_URL = "rtsp://localhost:8554/siyi_aruco"
CAMERA_URL = "rtsp://127.0.0.1:8554/my_camera" 

GPS_PORT = 5555
STREAM_W, STREAM_H = 1280, 720 
FIXED_FPS = 25   
FRAME_TIME_MS = 1.0 / FIXED_FPS 

# --- CẤU HÌNH GIMBAL ---
SIYI_IP = "192.168.168.14"
SIYI_PORT = 37260
TRACKING_ACTIVE = True # Tổng quyền bật/tắt tracking

# PID Gimbal
KP_YAW   = 0.15   
KP_PITCH = 0.25   
DEADZONE = 15     

# --- CẤU HÌNH CAMERA MATRIX ---
CAMERA_MATRIX = np.array([
    [717.14, 0.00, 664.29],
    [0.00, 717.88, 354.24],
    [0.00, 0.00, 1.00]
], dtype=float)

DIST_COEFFS = np.array([
    [-0.0764, 0.0742, -0.0013, 0.0019, -0.0176]
])

MARKER_SIZE = 0.099 # Mét

# --- HÀM TÍNH GÓC EULER ---
def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])

# --- FFMPEG ---
def get_ffmpeg_command(width, height, fps):
    return [
        'ffmpeg', '-y', '-f', 'rawvideo', '-vcodec', 'rawvideo', '-pix_fmt', 'bgr24', 
        '-s', f'{width}x{height}', '-r', str(fps), '-i', '-',
        '-c:v', 'h264_v4l2m2m', '-b:v', '2000k', '-pix_fmt', 'yuv420p', '-g', '30', '-bufsize', '1000k',   
        '-f', 'rtsp', RTSP_PUSH_URL
    ]

# --- SIYI GIMBAL CLASS ---
class SiyiGimbal:
    def __init__(self, ip, port):
        self.ip = ip; self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0; self.last_sent = 0

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
        if time.time() - self.last_sent < 0.05: return 
        self.last_sent = time.time()
        yaw_speed = max(min(int(yaw_speed), 100), -100)
        pitch_speed = max(min(int(pitch_speed), 100), -100)
        self.seq += 1
        payload = struct.pack('<bb', yaw_speed, pitch_speed) 
        msg = b'\x55\x66\x01' + struct.pack('<H', len(payload)) + struct.pack('<H', self.seq) + b'\x07' + payload
        try: self.sock.sendto(msg + self._append_crc16(msg)[-2:], (self.ip, self.port))
        except: pass

    def center(self):
        self.seq += 1
        payload = b'\x01'
        msg = b'\x55\x66\x01' + struct.pack('<H', len(payload)) + struct.pack('<H', self.seq) + b'\x08' + payload
        try: self.sock.sendto(msg + self._append_crc16(msg)[-2:], (self.ip, self.port))
        except: pass

    def stop(self):
        self.rotate(0, 0)

# --- CAMERA STREAM ---
class CameraStream:
    def __init__(self, src):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened(): print("❌ Camera Error!")
        self.ret, self.frame = False, None
        self.running = True; self.lock = threading.Lock()
        self.t = threading.Thread(target=self.update, args=(), daemon=True); self.t.start()
    def update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock: self.ret, self.frame = ret, frame
            else: time.sleep(0.01)
    def read(self):
        with self.lock: return self.ret, self.frame.copy() if self.frame is not None else None
    def stop(self): self.running = False; self.t.join(); self.cap.release()

# --- PUSHER ---
class StreamPusher:
    def __init__(self, cmd, w, h):
        self.process = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        self.queue = queue.Queue(maxsize=2)
        self.running = True; self.w, self.h = w, h
        self.t = threading.Thread(target=self.worker, args=(), daemon=True); self.t.start()
    def worker(self):
        while self.running:
            try:
                frame = self.queue.get(timeout=1)
                h, w = frame.shape[:2]
                if w != self.w or h != self.h: frame = cv2.resize(frame, (self.w, self.h))
                self.process.stdin.write(frame.tobytes()); self.queue.task_done()
            except: continue
    def write(self, frame):
        if self.running: 
            try: self.queue.put(frame, block=False)
            except: pass
    def stop(self): self.running = False; self.t.join(); self.process.stdin.close(); self.process.wait()

# --- GPS ---
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
    print(f">>> AUTO LANDING SYSTEM V1.0 (Hybrid Logic)...")
    print(f"   - Lock Altitude: {LANDING_ALTITUDE}m")
    print(f"   - Auto Center: {AUTO_CENTER_ON_LOCK}")
    
    cam = CameraStream(CAMERA_URL)
    time.sleep(2.0)
    if not cam.ret: print("❌ Camera not ready!"); cam.stop(); return
    
    pusher = StreamPusher(get_ffmpeg_command(STREAM_W, STREAM_H, FIXED_FPS), STREAM_W, STREAM_H)
    gimbal = SiyiGimbal(SIYI_IP, SIYI_PORT)
    gimbal.center() # Init Center
    
    try:
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshWinSizeStep = 10 
    except:
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        parameters = aruco.DetectorParameters()
        parameters.adaptiveThreshWinSizeStep = 10
    detector = aruco.ArucoDetector(aruco_dict, parameters) if hasattr(aruco, "ArucoDetector") else None

    # CSV Log
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_file = open(f"auto_land_log_{timestamp_str}.csv", mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Timestamp", "Algo_ms", "Loop_ms", "FPS", "X", "Y", "Z", "Roll", "Pitch", "Yaw", "Mode", "Gimbal_Yaw", "Gimbal_Pitch"])

    frame_count = 0; start_time_fps = time.perf_counter(); fps_val = 0; last_loop_duration = 0
    center_screen = (STREAM_W // 2, STREAM_H // 2)
    
    last_corners = None; last_ids = None
    last_yaw_cmd = 0; last_pitch_cmd = 0
    detect_count = 0
    
    # TRẠNG THÁI HỆ THỐNG
    is_locked = False # False = Tracking, True = Landing (Locked)
    
    print(">>> SYSTEM LIVE!")

    while True:
        loop_start = time.perf_counter()
        
        ret, frame = cam.read()
        if not ret or frame is None: time.sleep(0.001); continue

        t_start_algo = time.perf_counter()
        
        # --- 1. DETECT (Optimized) ---
        detect_count += 1
        should_detect = (detect_count % (SKIP_FRAME + 1) == 0)

        if should_detect:
            SMALL_W = int(STREAM_W * DETECT_SCALE); SMALL_H = int(STREAM_H * DETECT_SCALE)
            small_frame = cv2.resize(frame, (SMALL_W, SMALL_H), interpolation=cv2.INTER_NEAREST)
            gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
            
            if detector: corners, ids, _ = detector.detectMarkers(gray)
            else: corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            
            if ids is not None and len(ids) > 0:
                corners = tuple(c / DETECT_SCALE for c in corners)
                last_corners = corners; last_ids = ids
            else: last_ids = None
        else: corners = last_corners; ids = last_ids

        # --- 2. POSE & LOGIC ---
        pos_x = 0; pos_y = 0; pos_z = 0
        roll = 0; pitch = 0; yaw = 0
        current_mode = "SEARCH"
        
        if last_ids is not None and last_corners is not None:
            # 2a. Tính Pose
            try:
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(last_corners, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS)
                pos_x = tvecs[0][0][0]; pos_y = tvecs[0][0][1]; pos_z = tvecs[0][0][2]
                
                rmat, _ = cv2.Rodrigues(rvecs[0])
                euler = rotationMatrixToEulerAngles(rmat)
                roll, pitch, yaw = [math.degrees(x) for x in euler]

                cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, rvecs[0], tvecs[0], 0.05)
                aruco.drawDetectedMarkers(frame, last_corners, last_ids)
            except: pass

            # 2b. Logic Chuyển Mode (HYBRID CONTROL)
            if pos_z < LANDING_ALTITUDE:
                # -> VÀO CHẾ ĐỘ HẠ CÁNH (LOCKED)
                if not is_locked:
                    print(f">>> [AUTOPILOT] LOW ALTITUDE ({pos_z:.2f}m) -> LOCKING GIMBAL!")
                    is_locked = True
                    if AUTO_CENTER_ON_LOCK:
                        gimbal.center() # Khóa về 0
                    else:
                        gimbal.stop()   # Khóa tại chỗ
                
                current_mode = "LOCKED (LANDING)"
                last_yaw_cmd = 0
                last_pitch_cmd = 0
                
            else:
                # -> VÀO CHẾ ĐỘ TIẾP CẬN (TRACKING)
                if is_locked:
                    print(f">>> [AUTOPILOT] HIGH ALTITUDE ({pos_z:.2f}m) -> RESUME TRACKING!")
                    is_locked = False
                
                current_mode = "TRACKING"
                
                # Chỉ tính PID khi đang Tracking
                c = last_corners[0][0]
                cx = int((c[0][0] + c[2][0]) / 2); cy = int((c[0][1] + c[2][1]) / 2)
                err_x = cx - center_screen[0]; err_y = cy - center_screen[1]
                
                if abs(err_x) > DEADZONE: last_yaw_cmd = int(err_x * KP_YAW)
                else: last_yaw_cmd = 0
                if abs(err_y) > DEADZONE: last_pitch_cmd = int(-err_y * KP_PITCH)
                else: last_pitch_cmd = 0
                
                cv2.line(frame, center_screen, (cx, cy), (0, 0, 255), 2)
        
        else:
            current_mode = "SEARCHING"
            last_yaw_cmd = 0; last_pitch_cmd = 0
            # Reset Lock nếu mất dấu quá lâu? (Tùy chọn, hiện tại giữ nguyên trạng thái)

        # --- 3. GỬI LỆNH GIMBAL ---
        if TRACKING_ACTIVE:
            if not is_locked:
                gimbal.rotate(last_yaw_cmd, last_pitch_cmd)
            else:
                # Khi Locked, ta không gửi lệnh rotate nữa để Gimbal đứng yên
                # (Hoặc gửi rotate(0,0) liên tục để giữ cứng nếu cần)
                gimbal.rotate(0, 0)

        t_end_algo = time.perf_counter(); algo_ms = (t_end_algo - t_start_algo) * 1000

        # --- 4. OSD ---
        cv2.drawMarker(frame, center_screen, (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
        
        # Màu trạng thái: Xanh lá (Track), Đỏ (Lock/Land)
        mode_color = (0, 0, 255) if is_locked else (0, 255, 0)
        
        cv2.putText(frame, f"MODE: {current_mode}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_color, 2)
        cv2.putText(frame, f"ALT: {pos_z:.2f}m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        if last_ids is not None:
            # Hiển thị sai số điều khiển Drone (khi Locked, đây là số quan trọng nhất)
            c = last_corners[0][0]
            cx = int((c[0][0] + c[2][0]) / 2); cy = int((c[0][1] + c[2][1]) / 2)
            drone_err_x = cx - center_screen[0]
            drone_err_y = cy - center_screen[1]
            cv2.putText(frame, f"DRONE ERR: {drone_err_x}, {drone_err_y}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        pusher.write(frame)

        # --- 5. LOOP TIMING ---
        loop_end = time.perf_counter(); elapsed = loop_end - loop_start
        last_loop_duration = elapsed * 1000
        wait = FRAME_TIME_MS - elapsed
        if wait > 0: time.sleep(wait)
        
        frame_count += 1
        if frame_count >= 10:
            now = time.perf_counter(); fps_val = frame_count / (now - start_time_fps)
            start_time_fps = now; frame_count = 0
            try:
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                csv_writer.writerow([ts, f"{algo_ms:.1f}", f"{last_loop_duration:.1f}", f"{fps_val:.1f}", 
                                     f"{pos_x:.2f}", f"{pos_y:.2f}", f"{pos_z:.2f}", 
                                     f"{roll:.1f}", f"{pitch:.1f}", f"{yaw:.1f}", 
                                     current_mode, last_yaw_cmd, last_pitch_cmd])
            except: pass

    running_global = False; cam.stop(); pusher.stop(); gimbal.stop(); csv_file.close()

if __name__ == "__main__":
    t = threading.Thread(target=gps_thread, daemon=True); t.start()
    main()
