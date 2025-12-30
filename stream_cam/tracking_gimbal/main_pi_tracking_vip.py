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
# --- CẤU HÌNH TỐI ƯU HIỆU NĂNG ---
# ==========================================
DETECT_SCALE = 0.7  
SKIP_FRAME = 1      

# --- CẤU HÌNH HỆ THỐNG ---
RTSP_PUSH_URL = "rtsp://localhost:8554/siyi_aruco"
CAMERA_URL = "rtsp://127.0.0.1:8554/my_camera" 

GPS_PORT = 5555
STREAM_W, STREAM_H = 1280, 720 
FIXED_FPS = 30   
FRAME_TIME_MS = 1.0 / FIXED_FPS 

# --- CẤU HÌNH TRACKING (GIMBAL PID) ---
SIYI_IP = "192.168.168.14"
SIYI_PORT = 37260
TRACKING_ACTIVE = True

KP_YAW   = 0.15   
KP_PITCH = 0.25   
DEADZONE = 15     

# --- CẤU HÌNH CAMERA MATRIX ---
CAMERA_MATRIX = np.array([
    [717.14, 0.00, 664.29],
    [0.00, 717.88, 354.24],
    [0.00, 0.00, 1.00]
], dtype=np.float32)

DIST_COEFFS = np.array([
    [-0.0764, 0.0742, -0.0013, 0.0019, -0.0176]
], dtype=np.float32)

MARKER_SIZE = 0.1 # Mét

# =========================================================
# --- HÀM TÍNH TOÁN POSE (FIX SHAPE LỖI) ---
# =========================================================
def my_estimatePoseSingleMarkers(corners, marker_size, mtx, dist):
    marker_points = np.array([
        [-marker_size / 2, marker_size / 2, 0],
        [marker_size / 2, marker_size / 2, 0],
        [marker_size / 2, -marker_size / 2, 0],
        [-marker_size / 2, -marker_size / 2, 0]
    ], dtype=np.float32)

    rvecs = []
    tvecs = []
    
    for c in corners:
        _, r, t = cv2.solvePnP(marker_points, c, mtx, dist, False, cv2.SOLVEPNP_IPPE_SQUARE)
        # Reshape về (1, 3) để khớp với code cũ
        rvecs.append(r.reshape(1, 3))
        tvecs.append(t.reshape(1, 3))
        
    return rvecs, tvecs

# --- CÁC HÀM HỖ TRỢ KHÁC ---
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert(isRotationMatrix(R))
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

def get_ffmpeg_command(width, height, fps):
    return [
        'ffmpeg', '-y', '-f', 'rawvideo', '-vcodec', 'rawvideo', '-pix_fmt', 'bgr24', 
        '-s', f'{width}x{height}', '-r', str(fps), '-i', '-',
        '-c:v', 'h264_v4l2m2m', '-b:v', '2000k', '-pix_fmt', 'yuv420p', '-g', '30', '-bufsize', '1000k',   
        '-f', 'rtsp', RTSP_PUSH_URL
    ]

# --- CLASS SIYI GIMBAL ---
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
    def stop(self): self.rotate(0, 0)

# --- CLASS CAMERA STREAM (GSTREAMER MULTI-TRY) ---
class CameraStream:
    def __init__(self, src):
        self.src = src
        self.cap = None
        self.ret = False
        self.frame = None
        self.running = True
        self.mode = "NONE"
        
        # Danh sách các Pipeline để thử (Ưu tiên Software trước vì ổn định)
        pipelines = [
            # 1. Software Decoding (Ổn định nhất trên mọi hệ điều hành)
            (f"rtspsrc location={src} latency=0 ! "
             "rtph264depay ! h264parse ! avdec_h264 ! "
             "videoconvert ! video/x-raw,format=BGR ! appsink sync=false drop=true max-buffers=1"),
             
            # 2. Hardware Decoding (Raspberry Pi V4L2) - Nhanh nhưng kén driver
            (f"rtspsrc location={src} latency=0 ! "
             "rtph264depay ! h264parse ! v4l2h264dec capture-io-mode=4 ! "
             "videoconvert ! video/x-raw,format=BGR ! appsink sync=false drop=true max-buffers=1"),
             
            # 3. Auto Decoding (Hên xui)
            (f"rtspsrc location={src} latency=0 ! "
             "rtph264depay ! h264parse ! decodebin ! "
             "videoconvert ! video/x-raw,format=BGR ! appsink sync=false drop=true max-buffers=1")
        ]

        # Vòng lặp thử từng Pipeline
        for i, pipe in enumerate(pipelines):
            print(f">>> [Camera] Trying GStreamer Pipeline #{i+1}...")
            cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                # Đọc thử 1 frame để chắc chắn nó chạy
                ret, _ = cap.read()
                if ret:
                    self.cap = cap
                    self.mode = f"GSTREAMER_PIPE_{i+1}"
                    print(f"✅ GStreamer Success with Pipeline #{i+1}")
                    break
                else:
                    cap.release()
            else:
                print(f"❌ Pipeline #{i+1} failed.")

        # Nếu GStreamer tạch hết -> Fallback về Threaded Bufferless
        if self.cap is None:
            print("⚠️ All GStreamer pipelines failed! Switching to Standard Threaded Mode...")
            self.cap = cv2.VideoCapture(src)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.mode = "THREADED_BUFFERLESS"

        if not self.cap.isOpened():
            print("❌ FATAL: Cannot open camera source!")
            self.running = False
            return

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
            else:
                time.sleep(0.005) # Ngủ ngắn để tránh spam CPU khi mất tín hiệu

    def read(self):
        with self.lock:
            return self.ret, self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        self.t.join()
        self.cap.release()
        
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
    print(f">>> PI TRACKING VIP V3 (GStreamer Fix + Pose Shape Fix)...")
    
    # 1. Khởi tạo Camera
    cam = CameraStream(CAMERA_URL)
    
    # Chờ warm-up
    print("Waiting for camera stream...")
    time.sleep(2.0)
    
    if not cam.ret: print("❌ Camera not ready!"); cam.stop(); return
    print(f"✅ Camera Active Mode: {cam.mode}")
    
    pusher = StreamPusher(get_ffmpeg_command(STREAM_W, STREAM_H, FIXED_FPS), STREAM_W, STREAM_H)
    gimbal = SiyiGimbal(SIYI_IP, SIYI_PORT)
    gimbal.center()
    
    try:
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshWinSizeStep = 10 
    except:
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        parameters = aruco.DetectorParameters()
        parameters.adaptiveThreshWinSizeStep = 10
    
    detector = aruco.ArucoDetector(aruco_dict, parameters) if hasattr(aruco, "ArucoDetector") else None

    # Log CSV
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_file = open(f"track_vip_v3_log_{timestamp_str}.csv", mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Timestamp", "Algo_ms", "Loop_ms", "FPS", "Pos_X", "Pos_Y", "Pos_Z", "Roll", "Pitch", "Yaw", "Yaw_Cmd", "Pitch_Cmd"])

    frame_count = 0; start_time_fps = time.perf_counter(); fps_val = 0; last_loop_duration = 0 
    center_screen = (STREAM_W // 2, STREAM_H // 2)
    last_corners = None; last_ids = None; last_yaw_cmd = 0; last_pitch_cmd = 0
    detect_count = 0 
    
    print(">>> SYSTEM LIVE!")

    while True:
        loop_start_counter = time.perf_counter()
        
        # 1. Đọc ảnh (Tức thì từ Thread)
        ret, frame = cam.read()
        if not ret or frame is None:
            time.sleep(0.001); continue

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
                # Ép kiểu float32 cho solvePnP
                corners = tuple((c / DETECT_SCALE).astype(np.float32) for c in corners)
                last_corners = corners; last_ids = ids
            else: last_ids = None
        else: corners = last_corners; ids = last_ids

        # --- 2. POSE & PID ---
        is_tracking = False
        pos_x = 0.0; pos_y = 0.0; pos_z = 0.0
        roll = 0.0; pitch = 0.0; yaw = 0.0
        
        if last_ids is not None and last_corners is not None:
            is_tracking = True
            try:
                # Dùng hàm thay thế (đã sửa shape)
                rvecs, tvecs = my_estimatePoseSingleMarkers(last_corners, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS)
                
                pos_x = tvecs[0][0][0]; pos_y = tvecs[0][0][1]; pos_z = tvecs[0][0][2]
                
                rmat, _ = cv2.Rodrigues(rvecs[0])
                euler_angles = rotationMatrixToEulerAngles(rmat)
                roll, pitch, yaw = [math.degrees(x) for x in euler_angles]

                cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, rvecs[0], tvecs[0], 0.05)
                aruco.drawDetectedMarkers(frame, last_corners, last_ids)
                
            except Exception as e: 
                if frame_count % 60 == 0: print(f"⚠️ Pose Error: {e}")

            # PID Tracking
            c = last_corners[0][0]
            cx = int((c[0][0] + c[2][0]) / 2); cy = int((c[0][1] + c[2][1]) / 2)
            err_x = cx - center_screen[0]; err_y = cy - center_screen[1]
            
            if abs(err_x) > DEADZONE: last_yaw_cmd = int(err_x * KP_YAW)
            else: last_yaw_cmd = 0
            if abs(err_y) > DEADZONE: last_pitch_cmd = int(-err_y * KP_PITCH)
            else: last_pitch_cmd = 0
            
            cv2.line(frame, center_screen, (cx, cy), (0, 0, 255), 2)
        else:
            is_tracking = False; last_yaw_cmd = 0; last_pitch_cmd = 0

        # Gimbal
        if TRACKING_ACTIVE: gimbal.rotate(last_yaw_cmd, last_pitch_cmd)

        t_end_algo = time.perf_counter(); algo_ms = (t_end_algo - t_start_algo) * 1000

        # OSD
        cv2.drawMarker(frame, center_screen, (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
        status_text = "DET" if should_detect else "SKP"
        cv2.putText(frame, f"FPS: {int(fps_val)} | {status_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        if is_tracking:
            info_pos = f"X:{pos_x:.2f} Y:{pos_y:.2f} Z:{pos_z:.2f}"
            cv2.putText(frame, info_pos, (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"CMD: Y{last_yaw_cmd} P{last_pitch_cmd}", (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        pusher.write(frame)

        # Loop time
        loop_end_counter = time.perf_counter(); elapsed_sec = loop_end_counter - loop_start_counter
        last_loop_duration = elapsed_sec * 1000 
        wait_time = FRAME_TIME_MS - elapsed_sec
        if wait_time > 0: time.sleep(wait_time)
        
        frame_count += 1
        if frame_count >= 10:
            now = time.perf_counter(); fps_val = frame_count / (now - start_time_fps)
            start_time_fps = now; frame_count = 0
            try:
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                csv_writer.writerow([ts, f"{algo_ms:.2f}", f"{last_loop_duration:.2f}", f"{fps_val:.2f}", 
                                     f"{pos_x:.3f}", f"{pos_y:.3f}", f"{pos_z:.3f}", 
                                     f"{roll:.2f}", f"{pitch:.2f}", f"{yaw:.2f}", 
                                     last_yaw_cmd, last_pitch_cmd])
            except: pass

    running_global = False; cam.stop(); pusher.stop(); gimbal.stop(); csv_file.close()

if __name__ == "__main__":
    t = threading.Thread(target=gps_thread, daemon=True); t.start()
    main()
