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
# --- 1. CONFIGURATION MANAGEMENT ---
# ==========================================
class Config:
    DEFAULTS = {
        "system": {
            "hardware_profile": "pi4", # [OPTIONS]: pi4, pi5, jetson_orin, cpu
            "ffmpeg_codec": "auto",    # "auto" để code tự chọn theo hardware_profile
            "detect_scale": 0.7,
            "skip_frame": 0,       
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
            "marker_size": 0.156
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
                    file_data = json.load(f)
                    for section, values in file_data.items():
                        if section in self.data:
                            self.data[section].update(values)
                        else:
                            self.data[section] = values
                logging.info(f"Loaded configuration from {config_path}")
            except Exception as e:
                logging.error(f"Failed to load config file: {e}. Using defaults.")
        else:
            logging.warning("Config file not found. Using default values.")

    def get(self, section, key):
        return self.data.get(section, {}).get(key)

# ==========================================
# --- 2. LOGGING SETUP ---
# ==========================================
def setup_logging():
    log_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(threadName)s - %(message)s')
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(log_formatter)
    root_logger.addHandler(console_handler)

# ==========================================
# --- 3. HELPER CLASSES ---
# ==========================================

class CSVDataLogger(threading.Thread):
    def __init__(self, prefix="benchmark_log"):
        super().__init__(daemon=True, name="CSVLoggerThread")
        self.queue = queue.Queue()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"{prefix}_{timestamp}.csv"
        self.header = [
            "Timestamp", "Frame_ID", "B3_Infer_ms", "C3_Algo_ms", "C4_Loop_ms", 
            "FPS_Inst", "D1_Detect", "Cam_X", "Cam_Y", "Cam_Z", 
            "Cam_Roll", "Cam_Pitch", "Cam_Yaw"
        ]
        self.running = True
        try:
            with open(self.filename, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.header)
            logging.info(f"CSV Logger initialized: {self.filename}")
        except Exception as e:
            logging.error(f"Failed to create CSV file: {e}")

    def log(self, data):
        if self.running: self.queue.put(data)

    def run(self):
        with open(self.filename, mode='a', newline='') as f:
            writer = csv.writer(f)
            while self.running or not self.queue.empty():
                try:
                    data = self.queue.get(timeout=1.0)
                    writer.writerow(data)
                    self.queue.task_done()
                except queue.Empty: continue
                except Exception: pass
    def stop(self):
        self.running = False
        self.join()

class FFmpegStreamer:
    def __init__(self, push_url, width, height, fps, codec="libx264"):
        self.width = width
        self.height = height
        self.target_size = (width, height)
        self.running = True
        self.queue = queue.Queue(maxsize=2)
        
        logging.info(f"FFmpeg Streamer Init | Codec: {codec}")

        self.cmd = [
            'ffmpeg', '-y', '-f', 'rawvideo', '-vcodec', 'rawvideo', 
            '-pix_fmt', 'bgr24', '-s', f'{width}x{height}', '-r', str(fps), 
            '-i', '-', 
            '-c:v', codec,
            '-pix_fmt', 'yuv420p', 
            '-g', '30', '-preset', 'ultrafast', 
            '-b:v', '2000k',
            '-loglevel', 'error',       
            '-f', 'rtsp', '-rtsp_transport', 'udp', push_url  
        ]

        if "libx264" in codec:
            idx = self.cmd.index('-b:v')
            self.cmd.insert(idx, 'zerolatency')
            self.cmd.insert(idx, '-tune')
        
        try:
            self.process = subprocess.Popen(self.cmd, stdin=subprocess.PIPE, bufsize=0)
        except Exception as e:
            logging.critical(f"FFmpeg failed: {e}")
            self.running = False
            return
        
        self.thread = threading.Thread(target=self._worker, daemon=True, name="StreamerThread")
        self.thread.start()

    def _worker(self):
        while self.running:
            try:
                frame = self.queue.get(timeout=1)
                if frame.shape[1] != self.width or frame.shape[0] != self.height:
                    frame_out = cv2.resize(frame, self.target_size, interpolation=cv2.INTER_LINEAR)
                else:
                    frame_out = frame
                self.process.stdin.write(frame_out.tobytes())
                self.queue.task_done()
            except queue.Empty: continue
            except Exception: self.running = False

    def write(self, frame):
        if not self.running: return
        try: self.queue.put_nowait(frame) 
        except queue.Full: pass 

    def stop(self):
        self.running = False
        if hasattr(self, 'thread'): self.thread.join()
        if hasattr(self, 'process') and self.process.stdin:
            self.process.stdin.close()
            self.process.wait()

class GStreamerCamera:
    """
    Class Camera Universal - Hỗ trợ 4 profile: pi4, pi5, jetson_orin, cpu
    """
    def __init__(self, source, hw_profile="cpu"):
        
        logging.info(f"Initializing Camera with Profile: [{hw_profile.upper()}]")
        
        # --- CASE 1: NVIDIA JETSON (Nano / Orin) ---
        if hw_profile == "jetson_orin":
            # Dùng nvv4l2decoder (Hardware) -> nvvidconv (Convert màu HW)
            decoder_pipeline = (
                "rtspsrc location={} latency=0 ! "
                "rtph264depay ! h264parse ! "
                "nvv4l2decoder ! "
                "nvvidconv ! video/x-raw, format=(string)BGRx ! "
                "videoconvert ! video/x-raw, format=(string)BGR ! "
                "appsink sync=false drop=true max-buffers=1"
            ).format(source)
            
        # --- CASE 2: RASPBERRY PI 4 (Có Hardware VPU H264) ---
        elif hw_profile == "pi4":
            # Dùng v4l2h264dec (Hardware)
            decoder_pipeline = (
                "rtspsrc location={} latency=0 ! "
                "rtph264depay ! h264parse ! "
		"v4l2h264dec capture-io-mode=4 ! "    # Giải nén bằng HW
		"v4l2convert capture-io-mode=4 ! "    # Chuyển màu cũng bằng HW
                #"avdec_h264 ! "
                #"queue max-size-buffers=1 leaky=downstream ! "
                "video/x-raw, format=(string)BGR ! "
                "appsink sync=false drop=true max-buffers=1"
            ).format(source)

        # --- CASE 3: RASPBERRY PI 5 (Bỏ HW H264, dùng CPU) ---
        elif hw_profile == "pi5":
            # Pi 5 CPU rất mạnh, dùng avdec_h264 là ổn định nhất.
            decoder_pipeline = (
                "rtspsrc location={} latency=0 ! "
                "rtph264depay ! h264parse ! "
                "avdec_h264 ! "
                "queue max-size-buffers=1 leaky=downstream ! "
                "videoconvert ! video/x-raw, format=(string)BGR ! "
                "appsink sync=false drop=true max-buffers=1"
            ).format(source)

        # --- CASE 4: PC / LAPTOP / CPU GENERIC ---
        else:
            # Fallback cho profile "cpu" hoặc nhập sai
            decoder_pipeline = (
                "rtspsrc location={} latency=0 ! "
                "rtph264depay ! h264parse ! avdec_h264 ! "
                "queue max-size-buffers=1 leaky=downstream ! "
                "videoconvert ! video/x-raw, format=(string)BGR ! "
                "appsink sync=false drop=true max-buffers=1"
            ).format(source)

        self.cap = cv2.VideoCapture(decoder_pipeline, cv2.CAP_GSTREAMER)
        
        # Cơ chế tự động Fallback (nếu pipeline trên lỗi thì về CPU thuần)
        if not self.cap.isOpened():
            logging.warning(f"Pipeline for {hw_profile} failed! Switching to generic CPU pipeline...")
            cpu_pipeline = (
                "rtspsrc location={} latency=0 ! "
                "rtph264depay ! h264parse ! avdec_h264 ! "
                "videoconvert ! video/x-raw, format=(string)BGR ! "
                "appsink sync=false drop=true max-buffers=1"
            ).format(source)
            self.cap = cv2.VideoCapture(cpu_pipeline, cv2.CAP_GSTREAMER)

        # Fallback cuối cùng: Native OpenCV
        if not self.cap.isOpened():
            logging.error("GStreamer failed completely. Fallback to native OpenCV.")
            self.cap = cv2.VideoCapture(source)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.ret = False
        self.latest_frame = None
        self.running = True
        self.lock = threading.Lock()
        
        self.thread = threading.Thread(target=self._update, daemon=True, name="CameraThread")
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
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0

    def _append_crc16(self, data):
        crc = 0
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
        return data + struct.pack('<H', crc)

    def set_angle(self, yaw, pitch):
        self.seq += 1
        yaw_val = int(yaw * 10)
        pitch_val = int(pitch * 10)
        payload = struct.pack('<hh', yaw_val, pitch_val)
        msg_head = b'\x55\x66\x01' + struct.pack('<H', len(payload)) + struct.pack('<H', self.seq) + b'\x0E'
        full_msg = msg_head + payload
        try:
            packet = self._append_crc16(full_msg)
            self.sock.sendto(packet, (self.ip, self.port))
        except Exception: pass
    def stop(self): self.sock.close()

# ==========================================
# --- 4. MAIN LOGIC (BENCHMARK MODE) ---
# ==========================================

def main():
    setup_logging()
    cfg = Config("config.json")
    
    # 1. Lấy Hardware Profile
    hw_profile = cfg.get('system', 'hardware_profile')
    if not hw_profile: hw_profile = "cpu"

    # 2. Tự động chọn Codec nếu config là "auto"
    ffmpeg_codec = cfg.get('system', 'ffmpeg_codec')
    if ffmpeg_codec == "auto" or not ffmpeg_codec:
        if hw_profile == "pi4":
            ffmpeg_codec = "h264_v4l2m2m" # Pi 4 có HW encoder
        else:
            ffmpeg_codec = "libx264"     # Pi 5, Jetson, CPU dùng libx264 cho an toàn
            
    logging.info(f">>> STARTING (Profile: {hw_profile}, FFmpeg: {ffmpeg_codec})...")

    sys_conf = cfg.data['system']
    conn_conf = cfg.data['connection']
    trk_conf = cfg.data['tracking']
    cam_mtx = np.array(cfg.data['camera_matrix']['matrix'], dtype=np.float32)
    dist_coeffs = np.array(cfg.data['camera_matrix']['dist_coeffs'], dtype=np.float32)

    stream_w = int(sys_conf['input_width'] * sys_conf['stream_scale'])
    stream_h = int(sys_conf['input_height'] * sys_conf['stream_scale'])
    frame_time_ms = 1.0 / sys_conf['target_fps']

    # Khởi tạo Camera với Profile cụ thể
    camera = GStreamerCamera(conn_conf['camera_source'], hw_profile=hw_profile)
    time.sleep(2.0)
    
    if not camera.ret:
        logging.critical("Camera not ready! Exiting.")
        camera.stop()
        return

    pusher = FFmpegStreamer(conn_conf['rtsp_push_url'], stream_w, stream_h, sys_conf['target_fps'], codec=ffmpeg_codec)
    gimbal = SiyiGimbalController(conn_conf['siyi_ip'], conn_conf['siyi_port'])
    
    for _ in range(3):
        gimbal.set_angle(0, -90) 
        time.sleep(0.05)
    last_gimbal_reset_time = time.time()

    logger = CSVDataLogger()
    logger.start()

    try:
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        aruco_params = aruco.DetectorParameters()
        aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX 
        detector = aruco.ArucoDetector(aruco_dict, aruco_params)
    except AttributeError:
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        aruco_params = aruco.DetectorParameters_create()
        aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        detector = None 

    frame_count = 0
    total_frames_processed = 0 
    total_detected_frames = 0
    all_loop_times = [] 
    
    start_time_global = time.perf_counter()
    fps = 0.0
    detect_counter = 0

    logging.info(">>> LOOP STARTED")

    try:
        while True:
            loop_start = time.perf_counter()
            ret, frame = camera.read()
            if not ret or frame is None:
                time.sleep(0.001)
                continue

            c3_start = time.perf_counter()
            detect_counter += 1
            should_detect = (detect_counter % (sys_conf['skip_frame'] + 1) == 0)
            
            cam_x, cam_y, cam_z = 0, 0, 0
            cam_roll, cam_pitch, cam_yaw = 0, 0, 0
            is_detected = 0
            b3_infer_ms = 0

            if should_detect:
                scale = sys_conf['detect_scale']
                small_frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
                gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
                b3_start = time.perf_counter()
                
                if detector:
                    corners, ids, _ = detector.detectMarkers(gray)
                else:
                    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
                
                b3_infer_ms = (time.perf_counter() - b3_start) * 1000
                total_frames_processed += 1

                if ids is not None and len(ids) > 0:
                    total_detected_frames += 1
                    is_detected = 1
                    corners = tuple((c / scale).astype(np.float32) for c in corners)
                    marker_points = np.array([
                        [-trk_conf['marker_size']/2, trk_conf['marker_size']/2, 0],
                        [trk_conf['marker_size']/2, trk_conf['marker_size']/2, 0],
                        [trk_conf['marker_size']/2, -trk_conf['marker_size']/2, 0],
                        [-trk_conf['marker_size']/2, -trk_conf['marker_size']/2, 0]
                    ], dtype=np.float32)
                    _, rvec, tvec = cv2.solvePnP(marker_points, corners[0], cam_mtx, dist_coeffs, False, cv2.SOLVEPNP_IPPE_SQUARE)
                    cam_x, cam_y, cam_z = tvec[0][0], tvec[1][0], tvec[2][0]
                    rmat, _ = cv2.Rodrigues(rvec)
                    sy = math.sqrt(rmat[0,0] * rmat[0,0] +  rmat[1,0] * rmat[1,0])
                    if sy < 1e-6:
                        x = math.atan2(-rmat[1,2], rmat[1,1])
                        y = math.atan2(-rmat[2,0], sy)
                        z = 0
                    else:
                        x = math.atan2(rmat[2,1] , rmat[2,2])
                        y = math.atan2(-rmat[2,0], sy)
                        z = math.atan2(rmat[1,0], rmat[0,0])
                    cam_roll, cam_pitch, cam_yaw = math.degrees(x), math.degrees(y), math.degrees(z)
                    cv2.drawFrameAxes(frame, cam_mtx, dist_coeffs, rvec, tvec, 0.05)
                    aruco.drawDetectedMarkers(frame, corners, ids)
            
            c3_algo_ms = (time.perf_counter() - c3_start) * 1000

            if time.time() - last_gimbal_reset_time > 5.0:
                gimbal.set_angle(0, -90)
                last_gimbal_reset_time = time.time()

            if total_frames_processed > 0:
                d1_rate = (total_detected_frames / total_frames_processed) * 100
            else:
                d1_rate = 0.0

            cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"B3: {b3_infer_ms:.1f}ms", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"D1: {d1_rate:.1f}%", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            
            if is_detected:
                pos_text = f"Cam X: {cam_x:.2f} Y: {cam_y:.2f} Z: {cam_z:.2f}"
                cv2.putText(frame, pos_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

            pusher.write(frame)

            elapsed = time.perf_counter() - loop_start
            loop_ms = elapsed * 1000
            all_loop_times.append(loop_ms)
            wait_time = frame_time_ms - elapsed
            if wait_time > 0: time.sleep(wait_time)

            frame_count += 1
            if frame_count >= 10:
                now = time.perf_counter()
                fps = frame_count / (now - start_time_global)
                start_time_global = now
                frame_count = 0
                log_payload = [
                    datetime.now().strftime("%H:%M:%S.%f")[:-3], detect_counter,
                    f"{b3_infer_ms:.2f}", f"{c3_algo_ms:.2f}", f"{loop_ms:.2f}", f"{fps:.2f}",
                    is_detected, f"{cam_x:.3f}", f"{cam_y:.3f}", f"{cam_z:.3f}",
                    f"{cam_roll:.2f}", f"{cam_pitch:.2f}", f"{cam_yaw:.2f}"
                ]
                logger.log(log_payload)

    except KeyboardInterrupt: logging.info("User requested stop.")
    except Exception as e: logging.critical(f"Main Loop Crash: {e}")
    finally:
        camera.stop()
        pusher.stop()
        gimbal.stop()
        logger.stop()

        print("\n" + "="*40)
        print(">>> BENCHMARK REPORT SUMMARY <<<")
        print("="*40)
        if len(all_loop_times) > 0:
            avg_fps = 1000.0 / np.mean(all_loop_times)
            print(f"[B1] Average FPS:       {avg_fps:.2f} fps")
            sorted_times = sorted(all_loop_times, reverse=True) 
            idx_1_percent = int(len(sorted_times) * 0.01)
            worst_1_percent_time = sorted_times[idx_1_percent]
            low_1_fps = 1000.0 / worst_1_percent_time if worst_1_percent_time > 0 else 0
            print(f"[B2] 1% Low FPS:        {low_1_fps:.2f} fps")
        if total_frames_processed > 0:
            final_d1 = (total_detected_frames / total_frames_processed) * 100
            print(f"[D1] Detection Rate:    {final_d1:.2f}% ({total_detected_frames}/{total_frames_processed})")
        print("="*40 + "\n")
        logging.info(">>> SHUTDOWN COMPLETE")

if __name__ == "__main__":
    main()
