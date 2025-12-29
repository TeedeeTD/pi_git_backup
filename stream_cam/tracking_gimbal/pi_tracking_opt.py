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

# ==========================================
# --- CẤU HÌNH TỐI ƯU HIỆU NĂNG ---
# ==========================================
DETECT_SCALE = 0.7  # Thu nhỏ để detect nhanh
SKIP_FRAME = 1      # Detect 1 frame, nghỉ 1 frame

# --- CẤU HÌNH HỆ THỐNG ---
RTSP_PUSH_URL = "rtsp://localhost:8554/siyi_aruco"
CAMERA_URL = "rtsp://127.0.0.1:8554/my_camera" 

GPS_PORT = 5555
STREAM_W, STREAM_H = 1280, 720 
FIXED_FPS = 25   
FRAME_TIME_MS = 1.0 / FIXED_FPS 

# --- CẤU HÌNH TRACKING (GIMBAL PID) ---
SIYI_IP = "192.168.168.14"
SIYI_PORT = 37260
TRACKING_ACTIVE = True

# PID 
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

# --- FFMPEG OUTPUT COMMAND ---
def get_ffmpeg_command(width, height, fps):
    return [
        'ffmpeg', '-y', 
        '-f', 'rawvideo', '-vcodec', 'rawvideo', '-pix_fmt', 'bgr24', 
        '-s', f'{width}x{height}', '-r', str(fps),
        '-i', '-',
        '-c:v', 'h264_v4l2m2m', 
        '-b:v', '2000k',       
        '-pix_fmt', 'yuv420p', 
        '-g', '30',            
        '-bufsize', '1000k',   
        '-f', 'rtsp', RTSP_PUSH_URL
    ]

# --- CLASS SIYI GIMBAL ---
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
        if time.time() - self.last_sent < 0.05: return 
        self.last_sent = time.time()

        yaw_speed = max(min(int(yaw_speed), 100), -100)
        pitch_speed = max(min(int(pitch_speed), 100), -100)
        
        self.seq += 1
        payload = struct.pack('<bb', yaw_speed, pitch_speed) 
        header = b'\x55\x66' 
        ctrl = b'\x01'       
        length = struct.pack('<H', len(payload))
        seq_bytes = struct.pack('<H', self.seq)
        cmd_id = b'\x07'
        
        msg = header + ctrl + length + seq_bytes + cmd_id + payload
        msg_with_crc = self._append_crc16(msg)
        try: self.sock.sendto(msg_with_crc, (self.ip, self.port))
        except: pass

    def center(self):
        self.seq += 1
        cmd_id, payload = b'\x08', b'\x01'
        header, ctrl = b'\x55\x66', b'\x01'
        length = struct.pack('<H', len(payload))
        seq_bytes = struct.pack('<H', self.seq)
        msg = header + ctrl + length + seq_bytes + cmd_id + payload
        try: self.sock.sendto(msg + self._append_crc16(msg)[-2:], (self.ip, self.port))
        except: pass

    def stop(self):
        self.rotate(0, 0)

# --- CLASS CAMERA STREAM ---
class CameraStream:
    def __init__(self, src):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened(): print("❌ Camera Error!")
        else: print("✅ Camera Input: ACTIVE")
        self.ret, self.frame = False, None
        self.running = True
        self.lock = threading.Lock()
        self.t = threading.Thread(target=self.update, args=(), daemon=True)
        self.t.start()

    def update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.ret, self.frame = ret, frame
            else: time.sleep(0.01)

    def read(self):
        with self.lock:
            return self.ret, self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        self.t.join()
        self.cap.release()

# --- CLASS STREAM PUSHER ---
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
                if w != self.w or h != self.h: frame = cv2.resize(frame, (self.w, self.h))
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

# --- MAIN PROGRAM ---
def main():
    global running_global
    print(f">>> PI CM4 OPTIMIZED TRACKING V3.0 (Sync Timing)...")
    
    cam = CameraStream(CAMERA_URL)
    time.sleep(2.0)
    if not cam.ret: print("❌ Camera not ready!"); cam.stop(); return
    
    ffmpeg_cmd = get_ffmpeg_command(STREAM_W, STREAM_H, FIXED_FPS)
    pusher = StreamPusher(ffmpeg_cmd, STREAM_W, STREAM_H)
    
    gimbal = SiyiGimbal(SIYI_IP, SIYI_PORT)
    gimbal.center()
    print(f">>> Gimbal Connected: {SIYI_IP}")

    try:
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshWinSizeStep = 10 
    except:
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        parameters = aruco.DetectorParameters()
        parameters.adaptiveThreshWinSizeStep = 10
    
    detector = aruco.ArucoDetector(aruco_dict, parameters) if hasattr(aruco, "ArucoDetector") else None

    # --- CSV Logging ---
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_file = open(f"pi_track_opt_log_{timestamp_str}.csv", mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Timestamp", "Algo_ms", "Loop_ms", "FPS", "Dist_m", "ErrX", "ErrY", "Yaw_Cmd", "Pitch_Cmd"])

    # --- BIẾN ĐO THỜI GIAN ---
    frame_count = 0
    start_time_fps = time.perf_counter()
    fps_val = 0
    last_loop_duration = 0 # Loop_ms hiển thị
    
    center_screen = (STREAM_W // 2, STREAM_H // 2)
    last_corners = None
    last_ids = None
    last_yaw_cmd = 0
    last_pitch_cmd = 0
    
    # Biến nội bộ logic tối ưu
    detect_count = 0 
    
    print(">>> SYSTEM LIVE!")

    while True:
        # 1. Bắt đầu đếm giờ Loop bằng perf_counter (Chính xác cao)
        loop_start_counter = time.perf_counter()

        ret, frame = cam.read()
        if not ret or frame is None:
            time.sleep(0.001)
            continue

        # 2. XỬ LÝ (Algo)
        t_start_algo = time.perf_counter()
        
        # --- LOGIC TỐI ƯU (SCALE + SKIP) ---
        detect_count += 1
        should_detect = (detect_count % (SKIP_FRAME + 1) == 0)

        if should_detect:
            # Thu nhỏ
            SMALL_W = int(STREAM_W * DETECT_SCALE)
            SMALL_H = int(STREAM_H * DETECT_SCALE)
            small_frame = cv2.resize(frame, (SMALL_W, SMALL_H), interpolation=cv2.INTER_NEAREST)
            gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
            
            # Detect
            if detector: corners, ids, _ = detector.detectMarkers(gray)
            else: corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            
            # Scale ngược
            if ids is not None and len(ids) > 0:
                corners = tuple(c / DETECT_SCALE for c in corners)
                last_corners = corners
                last_ids = ids
            else:
                last_ids = None
        else:
            # Dùng lại kết quả cũ
            corners = last_corners
            ids = last_ids

        # --- LOGIC ĐIỀU KHIỂN & 3D POSE ---
        is_tracking = False
        dist_val = 0.0
        err_x, err_y = 0, 0
        
        if last_ids is not None and last_corners is not None:
            is_tracking = True
            try:
                # Tính Pose
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(last_corners, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS)
                dist_val = tvecs[0][0][2]
                aruco.drawDetectedMarkers(frame, last_corners, last_ids)
            except: pass

            # PID
            c = last_corners[0][0]
            cx = int((c[0][0] + c[2][0]) / 2)
            cy = int((c[0][1] + c[2][1]) / 2)
            
            err_x = cx - center_screen[0]
            err_y = cy - center_screen[1]
            
            if abs(err_x) > DEADZONE: last_yaw_cmd = int(err_x * KP_YAW)
            else: last_yaw_cmd = 0
            
            if abs(err_y) > DEADZONE: last_pitch_cmd = int(-err_y * KP_PITCH)
            else: last_pitch_cmd = 0
            
            cv2.line(frame, center_screen, (cx, cy), (0, 0, 255), 2)
        else:
            is_tracking = False
            last_yaw_cmd = 0
            last_pitch_cmd = 0

        # Gửi lệnh Gimbal
        if TRACKING_ACTIVE:
            gimbal.rotate(last_yaw_cmd, last_pitch_cmd)

        t_end_algo = time.perf_counter()
        algo_ms = (t_end_algo - t_start_algo) * 1000

        # --- OSD ---
        cv2.drawMarker(frame, center_screen, (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
        
        status_text = "DET" if should_detect else "SKP"
        # Dòng 1: FPS
        cv2.putText(frame, f"FPS: {int(fps_val)} | {status_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # Dòng 2: Algo time
        cv2.putText(frame, f"Algo: {algo_ms:.1f}ms", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # Dòng 3: Loop time (Thời gian xử lý thực tế của vòng trước)
        cv2.putText(frame, f"Loop: {last_loop_duration:.1f}ms", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        if is_tracking:
            cv2.putText(frame, f"CMD: Y{last_yaw_cmd} P{last_pitch_cmd}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        pusher.write(frame)

        # 4. TÍNH TOÁN LOOP TIME & NGỦ
        loop_end_counter = time.perf_counter()
        elapsed_sec = loop_end_counter - loop_start_counter
        last_loop_duration = elapsed_sec * 1000 # Lưu lại để hiển thị vòng sau
        
        wait_time = FRAME_TIME_MS - elapsed_sec
        if wait_time > 0:
            time.sleep(wait_time)
        
        # 5. TÍNH FPS (TRUNG BÌNH MỖI 10 FRAME)
        frame_count += 1
        if frame_count >= 10:
            now = time.perf_counter()
            fps_val = frame_count / (now - start_time_fps)
            start_time_fps = now
            frame_count = 0
            
            # Log CSV
            try:
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                csv_writer.writerow([ts, f"{algo_ms:.2f}", f"{last_loop_duration:.2f}", f"{fps_val:.2f}", f"{dist_val:.2f}", err_x, err_y, last_yaw_cmd, last_pitch_cmd])
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
