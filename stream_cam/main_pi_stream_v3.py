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

# --- CẤU HÌNH ---
# Nếu MediaMTX chạy trên chính Pi này
RTSP_PUSH_URL = "rtsp://localhost:8554/siyi_aruco"
# RTSP Camera Input (Đổi IP cho đúng với IP của Siyi A8)
CAMERA_URL = "rtsp://192.168.168.13:8554/main_stream" 

GPS_PORT = 5555
STREAM_W, STREAM_H = 1280, 720 
FIXED_FPS = 30 
FRAME_TIME_MS = 1.0 / FIXED_FPS 

# --- FFMPEG COMMAND CHO UBUNTU SERVER ON PI ---
def get_ffmpeg_command(width, height, fps):
    return [
        'ffmpeg', '-y', 
        '-f', 'rawvideo', 
        '-vcodec', 'rawvideo',
        '-pix_fmt', 'bgr24',       # OpenCV trả về BGR
        '-s', f'{width}x{height}', 
        '-r', str(fps),
        '-i', '-',                 # Input từ Pipe
        
        # --- HARDWARE ENCODER CHO PI (V4L2M2M) ---
        '-c:v', 'h264_v4l2m2m',    # Sử dụng GPU Encoder
        '-b:v', '2000k',           # Bitrate 2Mbps (đủ nét cho 720p)
        '-pix_fmt', 'yuv420p',     # Bắt buộc cho encoder này
        '-g', str(fps),            # GOP size = FPS (1 keyframe/sec)
        '-bf', '0',                # No B-frames (giảm latency tối đa)
        
        # Output format
        '-f', 'rtsp', RTSP_PUSH_URL
    ]

# --- CLASS 1: ASYNC CAMERA READER ---
class CameraStream:
    def __init__(self, src):
        # Trên Ubuntu Server, ưu tiên dùng backend mặc định (FFmpeg backend của OpenCV)
        # Bỏ GStreamer pipeline phức tạp đi để tránh lỗi thư viện thiếu
        self.cap = cv2.VideoCapture(src)
        # Giảm buffer size xuống thấp nhất có thể
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            print(f"❌ Không thể kết nối tới Camera: {src}")
            self.running = False
        else:
            print(f"✅ Đã kết nối Camera: {src}")
            self.running = True

        self.ret = False
        self.frame = None
        self.lock = threading.Lock()
        self.t = threading.Thread(target=self.update, args=(), daemon=True)
        if self.running: self.t.start()

    def update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.ret = ret
                    self.frame = frame
            else:
                # Nếu mất tín hiệu, chờ nhẹ để không spam CPU
                time.sleep(0.01)

    def read(self):
        with self.lock:
            return self.ret, self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        if self.t.is_alive(): self.t.join()
        self.cap.release()

# --- CLASS 2: ASYNC STREAM PUSHER ---
class StreamPusher:
    def __init__(self, cmd, w, h):
        # bufsize=0: Unbuffered, đẩy đi ngay lập tức
        self.process = subprocess.Popen(cmd, stdin=subprocess.PIPE, bufsize=0)
        self.queue = queue.Queue(maxsize=2) # Hàng đợi max 2 frame để tránh lag
        self.running = True
        self.w, self.h = w, h
        self.t = threading.Thread(target=self.worker, args=(), daemon=True)
        self.t.start()

    def worker(self):
        while self.running:
            try:
                frame = self.queue.get(timeout=1)
                # Resize nếu kích thước sai (an toàn)
                if frame.shape[1] != self.w or frame.shape[0] != self.h:
                    frame = cv2.resize(frame, (self.w, self.h))
                
                self.process.stdin.write(frame.tobytes())
                self.queue.task_done()
            except queue.Empty: continue
            except BrokenPipeError:
                print("❌ FFMPEG Pipe Broken! Restarting might be needed.")
                break
            except Exception as e: 
                print(f"❌ Push Error: {e}")

    def write(self, frame):
        if not self.running: return
        try: 
            # put_nowait: Nếu queue đầy thì drop frame luôn (giữ realtime)
            self.queue.put_nowait(frame)
        except queue.Full: 
            pass 

    def stop(self):
        self.running = False
        self.t.join()
        try:
            self.process.stdin.close()
            self.process.wait(timeout=2)
        except:
            self.process.kill()

# --- GPS THREAD (Dummy) ---
running_global = True 
def gps_thread():
    # Giữ nguyên logic nhận GPS của bạn
    global running_global
    while running_global:
        time.sleep(1) 

# --- MAIN ---
def main():
    global running_global
    print(f">>> KHOI DONG SYSTEM TREN UBUNTU SERVER (PI CM4)...")

    cam = CameraStream(CAMERA_URL)
    # Chờ camera nóng máy
    for _ in range(10):
        time.sleep(0.2)
        if cam.ret: break
    
    if not cam.ret:
        print("❌ Camera chưa sẵn sàng. Kiểm tra lại IP/Kết nối!"); cam.stop(); return
    
    ffmpeg_cmd = get_ffmpeg_command(STREAM_W, STREAM_H, FIXED_FPS)
    print(f"Executing FFmpeg: {' '.join(ffmpeg_cmd)}") 
    pusher = StreamPusher(ffmpeg_cmd, STREAM_W, STREAM_H)

    # ArUco Setup
    try:
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)
    except AttributeError:
        # Fallback cho OpenCV cũ hơn (nếu có)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        parameters = aruco.DetectorParameters_create()
        detector = None

    print(">>> SYSTEM LIVE! Chạy lệnh 'htop' ở terminal khác để xem CPU load.")

    frame_count = 0
    start_time_fps = time.perf_counter()
    fps_val = 0
    
    while True:
        loop_start = time.perf_counter()

        # B1: Lấy ảnh
        ret, frame = cam.read()
        if not ret or frame is None:
            time.sleep(0.001)
            continue

        # B2: Xử lý ArUco
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if detector: 
            corners, ids, _ = detector.detectMarkers(gray)
        else: 
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

        # Vẽ FPS lên ảnh
        cv2.putText(frame, f"Pi FPS: {int(fps_val)}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # B3: Đẩy Output
        pusher.write(frame)

        # B4: Tính toán Sleep để giữ FPS ổn định
        loop_end = time.perf_counter()
        elapsed = loop_end - loop_start
        wait = FRAME_TIME_MS - elapsed
        if wait > 0:
            time.sleep(wait)
            
        # Tính FPS thực tế
        frame_count += 1
        if frame_count >= 30: 
            now = time.perf_counter()
            fps_val = frame_count / (now - start_time_fps)
            start_time_fps = now
            frame_count = 0
        
    running_global = False
    cam.stop()
    pusher.stop()

if __name__ == "__main__":
    t = threading.Thread(target=gps_thread, daemon=True)
    t.start()
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopping...")
        running_global = False
