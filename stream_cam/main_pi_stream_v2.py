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
RTSP_PUSH_URL = "rtsp://localhost:8554/siyi_aruco"
CAMERA_URL = "rtsp://127.0.0.1:8554/my_camera" 

GPS_PORT = 5555
STREAM_W, STREAM_H = 1280, 720 
FIXED_FPS = 15 
FRAME_TIME_MS = 1.0 / FIXED_FPS 

# --- FFMPEG OUTPUT COMMAND (PI HARDWARE V4L2M2M) ---
def get_ffmpeg_command(width, height, fps):
    return [
        'ffmpeg', '-y', 
        '-f', 'rawvideo', 
        '-vcodec', 'rawvideo',
        '-pix_fmt', 'bgr24', 
        '-s', f'{width}x{height}', 
        '-r', str(fps),
        '-i', '-',
        
        # Hardware Encoder
        '-c:v', 'h264_v4l2m2m', 
        
        # Giảm Bitrate xuống 2000k để giảm tải
        '-b:v', '2000k', 
        
        '-pix_fmt', 'yuv420p', 
        '-g', '30',            # Tăng GOP lên 30 để giảm số lượng keyframe phải nén
        '-bufsize', '2000k',   
        '-f', 'rtsp', RTSP_PUSH_URL
    ]

# --- CLASS 1: ASYNC CAMERA READER (GSTREAMER HARDWARE DECODE) ---
class CameraStream:
    def __init__(self, src):
        # TỐI ƯU HÓA: Thay avdec_h264 (CPU) bằng v4l2h264dec (GPU)
        pipeline = (
            f"rtspsrc location={src} latency=0 ! "
            "rtph264depay ! h264parse ! v4l2h264dec capture-io-mode=4 ! "
            "videoconvert ! appsink sync=false drop=true max-buffers=1"
        )
        
        print(f">>> GStreamer HW Pipeline: {pipeline}")
        
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            print("⚠️ Hardware Decode thất bại, thử quay lại CPU Decode...")
            # Fallback về CPU nếu Hardware lỗi
            pipeline_cpu = (
                f"rtspsrc location={src} latency=0 ! "
                "rtph264depay ! h264parse ! avdec_h264 ! "
                "videoconvert ! appsink sync=false drop=true max-buffers=1"
            )
            self.cap = cv2.VideoCapture(pipeline_cpu, cv2.CAP_GSTREAMER)
        
        if self.cap.isOpened():
            print("✅ GStreamer Input: ACTIVE")
            self.running = True
        else:
            print("❌ LỖI: Không thể mở Camera!")
            self.running = False

        self.ret = False
        self.frame = None
        self.lock = threading.Lock()
        
        if self.running:
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
                time.sleep(0.005)

    def read(self):
        with self.lock:
            return self.ret, self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        if hasattr(self, 't'):
            self.t.join()
        self.cap.release()

# --- CLASS 2: ASYNC STREAM PUSHER ---
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
            except Exception as e: break

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
    print(f">>> PI CM4 - V19 (FULL HARDWARE PIPELINE)...")

    # 1. Khởi tạo Input
    cam = CameraStream(CAMERA_URL)
    time.sleep(2.0)
    if not cam.running: return
    
    # 2. Khởi tạo Output
    ffmpeg_cmd = get_ffmpeg_command(STREAM_W, STREAM_H, FIXED_FPS)
    pusher = StreamPusher(ffmpeg_cmd, STREAM_W, STREAM_H)

    # 3. ArUco
    try:
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        parameters = aruco.DetectorParameters_create()
    except:
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
        parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters) if hasattr(aruco, "ArucoDetector") else None

    # CSV Logging
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_file = open(f"pi_v19_log_{timestamp_str}.csv", mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Timestamp", "Algo_ms", "Loop_ms", "FPS", "Dropped_Output"])
    
    print(">>> STREAMING STARTED!")
    
    frame_count = 0
    start_time_fps = time.perf_counter()
    fps_val = 0
    last_loop_duration = 0
    
    while True:
        loop_start_counter = time.perf_counter()

        ret, frame = cam.read()
        if not ret or frame is None:
            time.sleep(0.001)
            continue

        t_start_algo = time.perf_counter()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if detector: corners, ids, _ = detector.detectMarkers(gray)
        else: corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

        cv2.drawMarker(frame, (STREAM_W//2, STREAM_H//2), (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
        
        t_end_algo = time.perf_counter()
        algo_ms = (t_end_algo - t_start_algo) * 1000 

        cv2.putText(frame, f"FPS: {int(fps_val)} (HW)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f"Loop: {last_loop_duration:.1f}ms", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

        pusher.write(frame)

        loop_end_counter = time.perf_counter()
        elapsed_sec = loop_end_counter - loop_start_counter
        last_loop_duration = elapsed_sec * 1000
        
        wait_time = FRAME_TIME_MS - elapsed_sec
        if wait_time > 0:
            time.sleep(wait_time)
            
        frame_count += 1
        if frame_count >= 10:
            now = time.perf_counter()
            fps_val = frame_count / (now - start_time_fps)
            start_time_fps = now
            frame_count = 0
        
        try:
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            q_size = pusher.queue.qsize()
            drop_status = "FULL" if q_size >= 2 else "OK"
            csv_writer.writerow([ts, f"{algo_ms:.2f}", f"{last_loop_duration:.2f}", f"{fps_val:.2f}", drop_status])
        except: pass

    running_global = False
    cam.stop()
    pusher.stop()
    csv_file.close()

if __name__ == "__main__":
    t = threading.Thread(target=gps_thread, daemon=True)
    t.start()
    main()
