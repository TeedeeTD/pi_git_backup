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
import queue
import binascii  # <--- [NEW] Thư viện tính CRC cực nhanh

from datetime import datetime

# ==========================================
# --- 1. CONFIGURATION MANAGEMENT ---
# ==========================================
class Config:
    """
    Class quản lý cấu hình hệ thống.
    """
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
                    self.data.update(file_data)
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

    file_handler = logging.FileHandler('system_error.log')
    file_handler.setLevel(logging.WARNING)
    file_handler.setFormatter(log_formatter)
    root_logger.addHandler(file_handler)

# ==========================================
# --- 3. HELPER CLASSES ---
# ==========================================

class CSVDataLogger(threading.Thread):
    def __init__(self, prefix="track_log"):
        super().__init__(daemon=True, name="CSVLoggerThread")
        self.queue = queue.Queue()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"{prefix}_{timestamp}.csv"
        self.header = [
            "Timestamp", "Algo_ms", "Loop_ms", "FPS", 
            "Body_X", "Body_Y", "Alt_Z", "Marker_Yaw",
            "Cam_X", "Cam_Y", "Cam_Z", "Cam_Roll", "Cam_Pitch", "Cam_Yaw"
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
        if self.running:
            self.queue.put(data)

    def run(self):
        with open(self.filename, mode='a', newline='') as f:
            writer = csv.writer(f)
            while self.running or not self.queue.empty():
                try:
                    data = self.queue.get(timeout=1.0)
                    writer.writerow(data)
                    self.queue.task_done()
                except queue.Empty:
                    continue
                except Exception as e:
                    logging.error(f"CSV Write Error: {e}")

    def stop(self):
        self.running = False
        self.join()

class FFmpegStreamer:
    def __init__(self, push_url, width, height, fps):
        self.width = width
        self.height = height
        self.target_size = (width, height)
        self.running = True
        self.queue = queue.Queue(maxsize=2)
        
        self.cmd = [
            'ffmpeg', '-y', 
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo', 
            '-pix_fmt', 'bgr24',
            '-s', f'{width}x{height}', 
            '-r', str(fps), 
            '-i', '-',
            '-c:v', 'h264_v4l2m2m', # Codec phần cứng (trên Pi/Jetson). Nếu lỗi, đổi thành 'libx264'
            '-b:v', '2000k',        
            '-pix_fmt', 'yuv420p',  
            '-g', '30',             
            '-preset', 'ultrafast', 
            '-tune', 'zerolatency', 
            '-rtsp_transport', 'udp',
            '-f', 'rtsp', push_url  
        ]
        
        try:
            self.process = subprocess.Popen(self.cmd, stdin=subprocess.PIPE, bufsize=0)
            logging.info(f"FFmpeg started streaming to {push_url}")
        except Exception as e:
            logging.critical(f"Failed to start FFmpeg: {e}")
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
            except queue.Empty:
                continue
            except BrokenPipeError:
                logging.error("FFmpeg Pipe broken! Restart required.")
                self.running = False
            except Exception as e:
                logging.error(f"Stream Worker Error: {e}")

    def write(self, frame):
        if not self.running: return
        try:
            self.queue.put_nowait(frame) 
        except queue.Full:
            pass

    def stop(self):
        self.running = False
        if hasattr(self, 'thread'): self.thread.join()
        if hasattr(self, 'process') and self.process.stdin:
            self.process.stdin.close()
            self.process.wait()

class GStreamerCamera:
    def __init__(self, source):
        gst_pipeline = (
            f"rtspsrc location={source} latency=0 ! " 
            "rtph264depay ! h264parse ! avdec_h264 ! " 
            "queue max-size-buffers=1 leaky=downstream ! "
            "videoconvert ! video/x-raw, format=(string)BGR ! " 
            "appsink sync=false drop=true max-buffers=1" 
        )
        
        logging.info(f"Connecting to Camera: {source}")
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            logging.warning("GStreamer failed. Fallback to standard capture.")
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
    """
    Class điều khiển Gimbal SIYI qua giao thức UDP SDK.
    """
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.last_sent_time = 0
        logging.info(f"Gimbal Controller initialized at {ip}:{port}")

    def _append_crc16(self, data):
        """
        [CẬP NHẬT] Tính toán Checksum CRC16 bằng binascii (Tối ưu tốc độ).
        Thay thế vòng lặp Python chậm chạp bằng thư viện C.
        SIYI SDK sử dụng CRC16-CCITT (Poly 0x1021, Init 0).
        """
        crc = binascii.crc_hqx(data, 0)
        return data + struct.pack('<H', crc)

    def rotate(self, yaw_speed, pitch_speed):
        if time.time() - self.last_sent_time < 0.05: 
            return

        self.last_sent_time = time.time()
        self.seq += 1
        
        y_val = max(min(int(yaw_speed), 100), -100)
        p_val = max(min(int(pitch_speed), 100), -100)
        
        payload = struct.pack('<bb', y_val, p_val)
        msg_head = b'\x55\x66\x01' + struct.pack('<H', len(payload)) + struct.pack('<H', self.seq) + b'\x07'
        full_msg = msg_head + payload
        
        try:
            packet = self._append_crc16(full_msg)
            # Không cần cộng thêm packet[-2:] nữa vì hàm _append_crc16 đã trả về full packet
            # Nhưng code cũ của bạn có đoạn cộng dư thừa `final_packet = full_msg + packet[-2:]`
            # -> Logic đúng phải là gửi packet đã có CRC.
            
            # [LƯU Ý]: Code cũ của bạn: packet = _append_crc16(full_msg) -> trả về msg+crc
            # Sau đó: final_packet = full_msg + packet[-2:] -> Lấy CRC nối vào msg lần nữa?
            # Hàm _append_crc16 đã trả về (data + crc). 
            # Nên ở đây chỉ cần gửi packet là đủ.
            
            self.sock.sendto(packet, (self.ip, self.port))
        except Exception as e:
            logging.warning(f"Gimbal Send Error: {e}")

    def center(self):
        self.seq += 1
        payload = b'\x01'
        msg_head = b'\x55\x66\x01' + struct.pack('<H', len(payload)) + struct.pack('<H', self.seq) + b'\x08'
        full_msg = msg_head + payload
        try:
            packet = self._append_crc16(full_msg)
            self.sock.sendto(packet, (self.ip, self.port))
            logging.info("Sent Center Command")
        except Exception as e:
            logging.error(f"Gimbal Center Error: {e}")

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
        except Exception as e:
            logging.error(f"Gimbal Set Angle Error: {e}")

    def stop(self):
        self.rotate(0, 0)
        self.sock.close()

# ==========================================
# --- 4. MAIN LOGIC ---
# ==========================================

def main():
    setup_logging()
    cfg = Config("config.json")
    
    logging.info(">>> SYSTEM STARTING (OPTIMIZED CRC & FIXED LOGIC)...")

    sys_conf = cfg.data['system']
    conn_conf = cfg.data['connection']
    trk_conf = cfg.data['tracking']
    cam_mtx = np.array(cfg.data['camera_matrix']['matrix'], dtype=np.float32)
    dist_coeffs = np.array(cfg.data['camera_matrix']['dist_coeffs'], dtype=np.float32)

    stream_w = int(sys_conf['input_width'] * sys_conf['stream_scale'])
    stream_h = int(sys_conf['input_height'] * sys_conf['stream_scale'])
    frame_time_ms = 1.0 / sys_conf['target_fps']

    camera = GStreamerCamera(conn_conf['camera_source'])
    time.sleep(2.0) 
    
    if not camera.ret:
        logging.critical("Camera not ready! Exiting.")
        camera.stop()
        return

    pusher = FFmpegStreamer(conn_conf['rtsp_push_url'], stream_w, stream_h, sys_conf['target_fps'])
    gimbal = SiyiGimbalController(conn_conf['siyi_ip'], conn_conf['siyi_port'])
    
    logging.info("Setting Gimbal to -90 degrees...")
    for _ in range(3):
        gimbal.set_angle(0, -90) 
        time.sleep(0.05)
    
    last_gimbal_reset_time = time.time()
    GIMBAL_RESET_INTERVAL = 5.0 

    logger = CSVDataLogger()
    logger.start()

    try:
        # Code cho OpenCV mới (4.7+)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        aruco_params = aruco.DetectorParameters()
        aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX 
        aruco_params.adaptiveThreshWinSizeStep = 10
        detector = aruco.ArucoDetector(aruco_dict, aruco_params)
    except AttributeError:
        # Fallback cho OpenCV cũ
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        aruco_params = aruco.DetectorParameters_create()
        try:
            aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        except:
            pass
        detector = None 

    frame_count = 0
    start_time_global = time.perf_counter()
    fps = 0.0
    
    center_x, center_y = sys_conf['input_width'] // 2, sys_conf['input_height'] // 2
    
    last_corners, last_ids = None, None
    detect_counter = 0
    
    logging.info(">>> LOOP STARTED")

    try:
        while True:
            loop_start = time.perf_counter()

            # 1. ĐỌC CAMERA
            ret, frame = camera.read()
            if not ret or frame is None:
                time.sleep(0.001)
                continue

            # 2. PHÁT HIỆN ARUCO
            algo_start = time.perf_counter()
            detect_counter += 1
            should_detect = (detect_counter % (sys_conf['skip_frame'] + 1) == 0)
            
            if should_detect:
                scale = sys_conf['detect_scale']
                small_frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
                gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
                
                if detector:
                    corners, ids, _ = detector.detectMarkers(gray)
                else:
                    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
                
                if ids is not None and len(ids) > 0:
                    corners = tuple((c / scale).astype(np.float32) for c in corners)
                    last_corners, last_ids = corners, ids
                else:
                    last_ids = None
            else:
                corners, ids = last_corners, last_ids

            # 3. TÍNH TOÁN POSE
            body_x, body_y, body_z = 0, 0, 0
            cam_x_raw, cam_y_raw, cam_z_raw = 0, 0, 0
            cam_roll, cam_pitch, cam_yaw = 0, 0, 0
            marker_yaw = 0
            is_tracking = False

            if last_ids is not None and last_corners is not None:
                is_tracking = True
                try:
                    marker_points = np.array([
                        [-trk_conf['marker_size']/2, trk_conf['marker_size']/2, 0],
                        [trk_conf['marker_size']/2, trk_conf['marker_size']/2, 0],
                        [trk_conf['marker_size']/2, -trk_conf['marker_size']/2, 0],
                        [-trk_conf['marker_size']/2, -trk_conf['marker_size']/2, 0]
                    ], dtype=np.float32)
                    
                    _, rvec, tvec = cv2.solvePnP(marker_points, last_corners[0], cam_mtx, dist_coeffs, False, cv2.SOLVEPNP_IPPE_SQUARE)
                    
                    body_x = -tvec[1][0]
                    body_y = tvec[0][0]
                    body_z = tvec[2][0]
                    
                    rmat, _ = cv2.Rodrigues(rvec)
                    
                    sy = math.sqrt(rmat[0,0] * rmat[0,0] +  rmat[1,0] * rmat[1,0])
                    singular = sy < 1e-6
                    if not singular:
                        x = math.atan2(rmat[2,1] , rmat[2,2])
                        y = math.atan2(-rmat[2,0], sy)
                        z = math.atan2(rmat[1,0], rmat[0,0])
                    else:
                        x = math.atan2(-rmat[1,2], rmat[1,1])
                        y = math.atan2(-rmat[2,0], sy)
                        z = 0
                    
                    cam_roll = math.degrees(x)
                    cam_pitch = math.degrees(y)
                    cam_yaw = math.degrees(z)
                    
                    cam_x_raw = tvec[0][0]
                    cam_y_raw = tvec[1][0]
                    cam_z_raw = tvec[2][0]

                    marker_yaw = math.degrees(math.atan2(rmat[1,0], rmat[0,0]))

                    cv2.drawFrameAxes(frame, cam_mtx, dist_coeffs, rvec, tvec, 0.05)
                    aruco.drawDetectedMarkers(frame, last_corners, last_ids)
                    
                    c = last_corners[0][0]
                    curr_cx = int((c[0][0] + c[2][0]) / 2) 
                    curr_cy = int((c[0][1] + c[2][1]) / 2) 
                    cv2.line(frame, (center_x, center_y), (curr_cx, curr_cy), (0, 0, 255), 2)
                    
                    info_text = f"X:{body_x:.2f}m Y:{body_y:.2f}m H:{body_z:.2f}m Yaw:{marker_yaw:.1f}"
                    cv2.putText(frame, info_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                except Exception as e:
                    logging.error(f"Pose Estimation Error: {e}")
            
            # 4. QUẢN LÝ GIMBAL (ĐÃ SỬA LỖI TỰ HỦY)
            current_time = time.time()
            if current_time - last_gimbal_reset_time > GIMBAL_RESET_INTERVAL:
                # [FIXED] CHỈ reset khi KHÔNG tracking. Nếu đang tracking mà reset thì mất dấu ngay.
                if not is_tracking:
                    # gimbal.set_angle(0, -90) # Comment dòng này lại để an toàn, hoặc chỉ bật khi cần
                    pass # Hiện tại disable để tránh tai nạn
                last_gimbal_reset_time = current_time
            
            algo_time_ms = (time.perf_counter() - algo_start) * 1000

            # 5. OSD & OUTPUT
            cv2.drawMarker(frame, (center_x, center_y), (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
            cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            pusher.write(frame)

            # 6. LOGGING
            elapsed = time.perf_counter() - loop_start
            loop_ms = elapsed * 1000
            wait_time = frame_time_ms - elapsed
            
            if wait_time > 0:
                time.sleep(wait_time)

            frame_count += 1
            if frame_count >= 10:
                now = time.perf_counter()
                fps = frame_count / (now - start_time_global)
                start_time_global = now
                frame_count = 0
                
                log_payload = [
                    datetime.now().strftime("%H:%M:%S.%f")[:-3],
                    f"{algo_time_ms:.2f}",
                    f"{loop_ms:.2f}",
                    f"{fps:.2f}",
                    f"{body_x:.3f}", f"{body_y:.3f}", f"{body_z:.3f}",   
                    f"{marker_yaw:.2f}",
                    f"{cam_x_raw:.3f}", f"{cam_y_raw:.3f}", f"{cam_z_raw:.3f}",
                    f"{cam_roll:.2f}", f"{cam_pitch:.2f}", f"{cam_yaw:.2f}"
                ]
                logger.log(log_payload)

    except KeyboardInterrupt:
        logging.info("User requested stop.")
    except Exception as e:
        logging.critical(f"Main Loop Crash: {e}")
    finally:
        camera.stop()
        pusher.stop()
        gimbal.stop()
        logger.stop()
        logging.info(">>> SHUTDOWN COMPLETE")

if __name__ == "__main__":
    main()
