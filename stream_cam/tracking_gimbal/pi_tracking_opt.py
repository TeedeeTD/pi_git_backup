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
DETECT_SCALE = 0.5  # Thu nhỏ 1/2 để detect nhanh
SKIP_FRAME = 1      # Detect 1 frame, nghỉ 1 frame

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

# PID 
KP_YAW   = 0.08   
KP_PITCH = 0.08   
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

MARKER_SIZE = 0.142 # Mét

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

# --- MAIN PROGRAM ---
def main():
    print(f">>> PI CM4 OPTIMIZED TRACKING V2.1 (Scale={DETECT_SCALE}, Skip={SKIP_FRAME})...")
    
    cam = CameraStream(CAMERA_URL)
    time.sleep(2.0)
    if not cam.ret: print("❌ Camera not ready!"); cam.stop(); return
    
    ffmpeg_cmd = get_ffmpeg_command(STREAM_W, STREAM_H, FIXED_FPS)
    pusher = StreamPusher(ffmpeg_cmd, STREAM_W, STREAM_H)
    
    gimbal = SiyiGimbal(SIYI_IP, SIYI_PORT)
    gimbal.center()
    print(f">>> Gimbal Connected: {SIYI_IP}")

    try:
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshWinSizeStep = 20 
    except:
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
        parameters = aruco.DetectorParameters()
        parameters.adaptiveThreshWinSizeStep = 20
    
    detector = aruco.ArucoDetector(aruco_dict, parameters) if hasattr(aruco, "ArucoDetector") else None

    frame_count = 0
    start_time_fps = time.perf_counter()
    fps_val = 0
    center_screen = (STREAM_W // 2, STREAM_H // 2)
    
    last_corners = None
    last_ids = None
    last_yaw_cmd = 0
    last_pitch_cmd = 0
    is_tracking = False

    print(">>> SYSTEM LIVE!")

    while True:
        loop_start = time.perf_counter()
        frame_count += 1

        ret, frame = cam.read()
        if not ret or frame is None:
            time.sleep(0.001); continue

        t_algo_start = time.perf_counter()
        
        # --- LOGIC TỐI ƯU ---
        should_detect = (frame_count % (SKIP_FRAME + 1) == 0)

        if should_detect:
            # 1. Thu nhỏ
            small_frame = cv2.resize(frame, None, fx=DETECT_SCALE, fy=DETECT_SCALE, interpolation=cv2.INTER_NEAREST)
            gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
            
            # 2. Detect
            if detector: corners, ids, _ = detector.detectMarkers(gray)
            else: corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            
            # 3. Scale ngược (Đã sửa lỗi Tuple)
            if ids is not None and len(ids) > 0:
                # FIX LỖI TẠI ĐÂY: Duyệt qua tuple và nhân vô hướng
                corners = tuple(c / DETECT_SCALE for c in corners)
                
                last_corners = corners
                last_ids = ids
                is_tracking = True
            else:
                last_ids = None
                is_tracking = False
        else:
            corners = last_corners
            ids = last_ids

        # --- LOGIC ĐIỀU KHIỂN ---
        yaw_cmd, pitch_cmd = 0, 0
        dist_val = 0.0
        
        if is_tracking and last_corners is not None:
            try:
                # Tính Pose
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(last_corners, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS)
                dist_val = tvecs[0][0][2]
                
                aruco.drawDetectedMarkers(frame, last_corners, last_ids)
                # cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, rvecs[0], tvecs[0], 0.05)
            except: pass

            # PID
            c = last_corners[0][0]
            cx = int((c[0][0] + c[2][0]) / 2)
            cy = int((c[0][1] + c[2][1]) / 2)
            
            err_x = cx - center_screen[0]
            err_y = cy - center_screen[1]

            if abs(err_x) > DEADZONE: yaw_cmd = int(err_x * KP_YAW)
            if abs(err_y) > DEADZONE: pitch_cmd = int(-err_y * KP_PITCH)
            
            last_yaw_cmd = yaw_cmd
            last_pitch_cmd = pitch_cmd
            
            cv2.line(frame, center_screen, (cx, cy), (0, 0, 255), 2)
        else:
            last_yaw_cmd = 0
            last_pitch_cmd = 0

        if TRACKING_ACTIVE:
            gimbal.rotate(last_yaw_cmd, last_pitch_cmd)

        t_algo_end = time.perf_counter()
        algo_ms = (t_algo_end - t_algo_start) * 1000
        
        cv2.drawMarker(frame, center_screen, (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
        
        status_text = "DETECT" if should_detect else "SKIP"
        cv2.putText(frame, f"FPS: {int(fps_val)} | {status_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"Algo: {algo_ms:.1f}ms", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if is_tracking:
            cv2.putText(frame, f"CMD: Y{last_yaw_cmd} P{last_pitch_cmd}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        pusher.write(frame)

        elapsed = time.perf_counter() - loop_start
        wait = FRAME_TIME_MS - elapsed
        if wait > 0: time.sleep(wait)
        
        if frame_count % 15 == 0:
            now = time.perf_counter()
            fps_val = 15 / (now - start_time_fps)
            start_time_fps = now

    cam.stop()
    pusher.stop()
    gimbal.stop()

if __name__ == "__main__":
    main()
