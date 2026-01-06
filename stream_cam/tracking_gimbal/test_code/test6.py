import sys
import time
import threading
import socket
import struct
import cv2
import cv2.aruco as aruco
import subprocess
import csv
import json
import logging
import os
import math
import numpy as np
from datetime import datetime
import queue

# ==========================================
# --- 1. CONFIGURATION ---
# ==========================================
class Config:
    DEFAULTS = {
        "system": {
            "detect_scale": 0.7,   
            "skip_frame": 1,       
            "input_width": 1280,   
            "input_height": 720,
            "stream_scale": 0.5,   
            "target_fps": 30,      
            "gps_port": 5555       
        },
        "connection": {
            "rtsp_push_url": "rtsp://localhost:8554/siyi_aruco",
            "camera_source": "rtsp://127.0.0.1:8554/my_camera",
            "siyi_ip": "192.168.168.14",
            "siyi_port": 37260           
        },
        "tracking": {
            "active": True,        
            "kp_yaw": 0.15,        
            "kp_pitch": 0.25,      
            "deadzone": 15,        
            "marker_size": 0.1     
        },
        "camera_matrix": {
            "matrix": [[717.14, 0, 664.29], [0, 717.88, 354.24], [0, 0, 1]],
            "dist_coeffs": [[-0.0764, 0.0742, -0.0013, 0.0019, -0.0176]]
        }
    }

    def __init__(self, config_path="config.json"):
        self.data = self.DEFAULTS
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r') as f:
                    self.data.update(json.load(f))
                logging.info(f"Loaded configuration from {config_path}")
            except Exception as e:
                logging.error(f"Failed to load config: {e}")

    def get(self, section, key):
        return self.data.get(section, {}).get(key)

# ==========================================
# --- 2. LOGGING ---
# ==========================================
def setup_logging():
    log_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(log_formatter)
    root_logger.addHandler(console_handler)

# ==========================================
# --- 3. HARDWARE ACCELERATED CLASSES ---
# ==========================================

class FFmpegStreamer:
    """
    HARDWARE ENCODING OUTPUT
    Sử dụng h264_v4l2m2m để nén bằng VideoCore VI GPU.
    """
    def __init__(self, push_url, width, height, fps):
        self.width = width
        self.height = height
        self.running = True
        self.queue = queue.Queue(maxsize=2)
        
        # Lệnh FFmpeg tối ưu hóa cho Pi CM4
        self.cmd = [
            'ffmpeg', '-y', 
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-pix_fmt', 'bgr24',       # OpenCV gửi vào là BGR
            '-s', f'{width}x{height}', 
            '-r', str(fps),
            '-i', '-',                 # Đọc từ Pipe
            '-c:v', 'h264_v4l2m2m',    # <--- HARDWARE ENCODER
            '-b:v', '2000k',           # Bitrate
            '-pix_fmt', 'yuv420p',     # Định dạng pixel chuẩn cho RTSP
            '-g', '15',                # Keyframe interval thấp (0.5s) để hồi phục nhanh
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',    # Ép độ trễ xuống thấp nhất
            '-rtsp_transport', 'udp',  # <--- Dùng UDP để giảm trễ truyền dẫn (như đã thảo luận)
            '-f', 'rtsp', push_url
        ]
        
        try:
            self.process = subprocess.Popen(self.cmd, stdin=subprocess.PIPE, bufsize=0)
            logging.info(f"FFmpeg Hardware Streamer started: UDP -> {push_url}")
        except Exception as e:
            logging.critical(f"FFmpeg Start Error: {e}")
            self.running = False
            return

        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def _worker(self):
        while self.running:
            try:
                frame = self.queue.get(timeout=1)
                self.process.stdin.write(frame.data)
                self.queue.task_done()
            except queue.Empty: continue
            except Exception: self.running = False

    def write(self, frame):
        if not self.running: return
        # Resize nếu cần thiết
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame_out = cv2.resize(frame, (self.width, self.height))
        else:
            frame_out = frame
        
        try: self.queue.put_nowait(frame_out)
        except queue.Full: pass

    def stop(self):
        self.running = False
        if hasattr(self, 'process') and self.process.stdin:
            self.process.stdin.close()
            self.process.wait()

class GStreamerCamera:
    """
    HARDWARE DECODING INPUT
    Sử dụng v4l2h264dec để giải mã bằng VideoCore VI GPU.
    """
    def __init__(self, source):
        # --- PIPELINE GIẢI MÃ PHẦN CỨNG ---
        # 1. rtspsrc: Nhận mạng (latency=0)
        # 2. v4l2h264dec: Giải mã bằng GPU (capture-io-mode=4 dùng DMABUF)
        # 3. videoconvert: Chuyển màu từ NV12 (của GPU) sang BGR (của OpenCV)
        #    Lưu ý: videoconvert vẫn dùng CPU nhưng nhẹ hơn nhiều so với giải mã full.
        gst_pipeline = (
            f"rtspsrc location={source} latency=0 ! "
            "rtph264depay ! h264parse ! "
            "v4l2h264dec capture-io-mode=4 ! "  # <--- HARDWARE DECODER
            "videoconvert ! "                   # Chuyển đổi màu
            "video/x-raw, format=(string)BGR ! "# Ép kiểu đầu ra cho OpenCV
            "appsink sync=false drop=true max-buffers=1"
        )
        
        logging.info(f"Starting Hardware Decoding Pipeline: {source}")
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            logging.critical("GStreamer Hardware Pipeline Failed! Check /dev/video* availability.")
            # Fallback nếu phần cứng lỗi (tùy chọn)
            self.cap = cv2.VideoCapture(source)

        self.ret = False
        self.latest_frame = None
        self.running = True
        self.lock = threading.Lock()
        
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def _update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.ret = ret
                    self.latest_frame = frame
            else:
                time.sleep(0.005)

    def read(self):
        with self.lock:
            if self.latest_frame is not None:
                return self.ret, self.latest_frame.copy()
            return False, None

    def stop(self):
        self.running = False
        self.thread.join()
        self.cap.release()

class SiyiGimbalController:
    # (Giữ nguyên logic UDP vì đã tối ưu)
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.last_sent_time = 0

    def _append_crc16(self, data):
        crc = 0
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
        return data + struct.pack('<H', crc)

    def rotate(self, yaw_speed, pitch_speed):
        if time.time() - self.last_sent_time < 0.05: return
        self.last_sent_time = time.time()
        self.seq += 1
        y_val = max(min(int(yaw_speed), 100), -100)
        p_val = max(min(int(pitch_speed), 100), -100)
        payload = struct.pack('<bb', y_val, p_val)
        msg_head = b'\x55\x66\x01' + struct.pack('<H', len(payload)) + struct.pack('<H', self.seq) + b'\x07'
        full_msg = msg_head + payload
        try:
            packet = self._append_crc16(full_msg)
            self.sock.sendto(full_msg + packet[-2:], (self.ip, self.port))
        except Exception: pass

    def center(self):
        self.seq += 1
        payload = b'\x01'
        msg_head = b'\x55\x66\x01' + struct.pack('<H', len(payload)) + struct.pack('<H', self.seq) + b'\x08'
        full_msg = msg_head + payload
        try:
            packet = self._append_crc16(full_msg)
            self.sock.sendto(full_msg + packet[-2:], (self.ip, self.port))
        except Exception: pass

    def stop(self):
        self.rotate(0, 0)
        self.sock.close()

class CSVDataLogger(threading.Thread):
    # (Giữ nguyên logic Logging)
    def __init__(self):
        super().__init__(daemon=True)
        self.queue = queue.Queue()
        self.filename = f"track_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.running = True
        try:
            with open(self.filename, 'w', newline='') as f:
                csv.writer(f).writerow(["Time", "Algo_ms", "Loop_ms", "FPS", "X", "Y", "Z", "Roll", "Pitch", "Yaw"])
        except: pass

    def log(self, data):
        if self.running: self.queue.put(data)

    def run(self):
        with open(self.filename, 'a', newline='') as f:
            writer = csv.writer(f)
            while self.running or not self.queue.empty():
                try:
                    data = self.queue.get(timeout=1.0)
                    writer.writerow(data)
                    self.queue.task_done()
                except queue.Empty: continue

    def stop(self):
        self.running = False
        self.join()

# ==========================================
# --- 4. MAIN LOGIC ---
# ==========================================
def main():
    setup_logging()
    cfg = Config("config.json")
    logging.info(">>> SYSTEM STARTING (FULL HARDWARE ACCELERATION)...")

    sys_conf = cfg.data['system']
    conn_conf = cfg.data['connection']
    trk_conf = cfg.data['tracking']
    cam_mtx = np.array(cfg.data['camera_matrix']['matrix'], dtype=np.float32)
    dist_coeffs = np.array(cfg.data['camera_matrix']['dist_coeffs'], dtype=np.float32)

    stream_w = int(sys_conf['input_width'] * sys_conf['stream_scale'])
    stream_h = int(sys_conf['input_height'] * sys_conf['stream_scale'])
    frame_time_ms = 1.0 / sys_conf['target_fps']

    # --- KHỞI TẠO VỚI PIPELINE MỚI ---
    camera = GStreamerCamera(conn_conf['camera_source'])
    time.sleep(2.0)
    
    if not camera.ret:
        logging.critical("Camera failed to start with Hardware Decoder. Exiting.")
        camera.stop()
        return

    pusher = FFmpegStreamer(conn_conf['rtsp_push_url'], stream_w, stream_h, sys_conf['target_fps'])
    gimbal = SiyiGimbalController(conn_conf['siyi_ip'], conn_conf['siyi_port'])
    gimbal.center()
    logger = CSVDataLogger()
    logger.start()

    # --- ARUCO SETUP ---
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    aruco_params = aruco.DetectorParameters()
    aruco_params.adaptiveThreshWinSizeStep = 10 
    detector = aruco.ArucoDetector(aruco_dict, aruco_params)

    # Variables
    frame_count = 0
    start_time_global = time.perf_counter()
    fps = 0.0
    center_x, center_y = sys_conf['input_width'] // 2, sys_conf['input_height'] // 2
    last_corners, last_ids = None, None
    detect_counter = 0

    try:
        while True:
            loop_start = time.perf_counter()

            # 1. Đọc Camera (Đã giải mã bằng GPU)
            ret, frame = camera.read()
            if not ret or frame is None:
                time.sleep(0.001)
                continue

            # 2. Thuật toán (Chạy trên CPU)
            algo_start = time.perf_counter()
            detect_counter += 1
            if detect_counter % (sys_conf['skip_frame'] + 1) == 0:
                scale = sys_conf['detect_scale']
                small_frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
                gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = detector.detectMarkers(gray)
                if ids is not None and len(ids) > 0:
                    corners = tuple((c / scale).astype(np.float32) for c in corners)
                    last_corners, last_ids = corners, ids
                else:
                    last_ids = None
            else:
                corners, ids = last_corners, last_ids

            # 3. Tính toán Pose & PID
            pos_x, pos_y, pos_z = 0, 0, 0
            yaw_out, pitch_out = 0, 0
            roll, pitch, yaw = 0, 0, 0

            if last_ids is not None and last_corners is not None:
                try:
                    marker_points = np.array([
                        [-trk_conf['marker_size']/2, trk_conf['marker_size']/2, 0],
                        [trk_conf['marker_size']/2, trk_conf['marker_size']/2, 0],
                        [trk_conf['marker_size']/2, -trk_conf['marker_size']/2, 0],
                        [-trk_conf['marker_size']/2, -trk_conf['marker_size']/2, 0]
                    ], dtype=np.float32)
                    _, rvec, tvec = cv2.solvePnP(marker_points, last_corners[0], cam_mtx, dist_coeffs, False, cv2.SOLVEPNP_IPPE_SQUARE)
                    pos_x, pos_y, pos_z = tvec.reshape(3)
                    
                    # Euler calc...
                    rmat, _ = cv2.Rodrigues(rvec)
                    sy = math.sqrt(rmat[0,0]**2 + rmat[1,0]**2)
                    roll, pitch, yaw = map(math.degrees, [
                        math.atan2(rmat[2,1], rmat[2,2]),
                        math.atan2(-rmat[2,0], sy),
                        math.atan2(rmat[1,0], rmat[0,0])
                    ])

                    # Draw & PID
                    cv2.drawFrameAxes(frame, cam_mtx, dist_coeffs, rvec, tvec, 0.05)
                    aruco.drawDetectedMarkers(frame, last_corners, last_ids)
                    
                    c = last_corners[0][0]
                    curr_cx = int((c[0][0] + c[2][0]) / 2)
                    curr_cy = int((c[0][1] + c[2][1]) / 2)
                    
                    if abs(curr_cx - center_x) > trk_conf['deadzone']:
                        yaw_out = int((curr_cx - center_x) * trk_conf['kp_yaw'])
                    if abs(curr_cy - center_y) > trk_conf['deadzone']:
                        pitch_out = int(-(curr_cy - center_y) * trk_conf['kp_pitch'])
                    
                    cv2.line(frame, (center_x, center_y), (curr_cx, curr_cy), (0, 0, 255), 2)
                except Exception: pass

            algo_time_ms = (time.perf_counter() - algo_start) * 1000

            # 4. Điều khiển & OSD
            if trk_conf['active']:
                gimbal.rotate(yaw_out, pitch_out)

            cv2.drawMarker(frame, (center_x, center_y), (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
            cv2.putText(frame, f"FPS: {int(fps)} | Algo: {int(algo_time_ms)}ms", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 5. Stream Output (Hardware Encode)
            pusher.write(frame)

            # 6. Loop Timing
            elapsed = time.perf_counter() - loop_start
            loop_ms = elapsed * 1000
            if frame_time_ms - elapsed > 0:
                time.sleep(frame_time_ms - elapsed)

            frame_count += 1
            if frame_count >= 10:
                fps = frame_count / (time.perf_counter() - start_time_global)
                start_time_global = time.perf_counter()
                frame_count = 0
                logger.log([datetime.now().strftime("%H:%M:%S.%f")[:-3], f"{algo_time_ms:.2f}", f"{loop_ms:.2f}", f"{fps:.2f}", f"{pos_x:.3f}", f"{pos_y:.3f}", f"{pos_z:.3f}", f"{roll:.2f}", f"{pitch:.2f}", f"{yaw:.2f}"])

    except KeyboardInterrupt: logging.info("Stopping...")
    except Exception as e: logging.critical(f"Crash: {e}")
    finally:
        camera.stop()
        pusher.stop()
        gimbal.stop()
        logger.stop()

if __name__ == "__main__":
    main()
