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
# --- 3. MATH HELPER (MA TRẬN QUAY) ---
# ==========================================
def get_body_position(tvec, g_roll, g_pitch, g_yaw):
    """
    Chuyển đổi vector tvec (Camera Frame) sang Body Frame
    bằng cách nhân với Ma trận quay nghịch đảo của Gimbal.
    Input: tvec (mét), Roll/Pitch/Yaw Gimbal (độ)
    Output: X(Tiến), Y(Phải), Z(Độ cao)
    """
    # Đổi sang radian
    r, p, y = map(math.radians, [g_roll, g_pitch, g_yaw])

    # Ma trận quay (Rotation Matrices)
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])

    # R_gimbal = Rz * Ry * Rx (Thứ tự xoay Yaw -> Pitch -> Roll)
    R_gimbal = Rz @ Ry @ Rx

    # Vector vị trí trong Camera Frame (OpenCV: X-Phải, Y-Xuống, Z-Trước)
    # Ta cần giữ nguyên vector này để nhân ma trận xoay
    P_camera = np.array([tvec[0], tvec[1], tvec[2]])

    # Nhân ma trận: P_body_raw = R * P_cam
    # P_body_raw là vector tvec ĐÃ ĐƯỢC BÙ GÓC XOAY, nhưng vẫn theo trục Camera gốc
    P_body_raw = np.dot(R_gimbal, P_camera)

    # Chuyển hệ trục từ "Gimbal Frame đã xoay" sang "NED Body Frame"
    # Sau khi bù góc quay, trục Z của Camera (Depth) sẽ trùng với trục X của Body (Tiến)
    
    final_x = P_body_raw[2]  # Trục Z cam -> Body X (Tiến)
    final_y = P_body_raw[0]  # Trục X cam -> Body Y (Phải)
    final_z = P_body_raw[1]  # Trục Y cam -> Body Z (Độ cao)

    return final_x, final_y, final_z

# ==========================================
# --- 4. CLASSES ---
# ==========================================

class CSVDataLogger(threading.Thread):
    def __init__(self, prefix="track_log"):
        super().__init__(daemon=True, name="CSVLoggerThread")
        self.queue = queue.Queue()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"{prefix}_{timestamp}.csv"
        
        # Header Log bao gồm cả Pose (Body Frame) và góc Gimbal thực tế
        self.header = ["Timestamp", "Algo_ms", "Loop_ms", "FPS", 
                       "Body_X", "Body_Y", "Alt_Z", "Marker_Yaw",
                       "Gimbal_R", "Gimbal_P", "Gimbal_Y"]
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
    # (Giữ nguyên class Streamer)
    def __init__(self, push_url, width, height, fps):
        self.width = width
        self.height = height
        self.running = True
        self.queue = queue.Queue(maxsize=2)
        
        self.cmd = [
            'ffmpeg', '-y', '-f', 'rawvideo', '-vcodec', 'rawvideo', '-pix_fmt', 'bgr24',
            '-s', f'{width}x{height}', '-r', str(fps), '-i', '-',
            '-c:v', 'h264_v4l2m2m', '-b:v', '2000k', '-pix_fmt', 'yuv420p',  
            '-g', '30', '-preset', 'ultrafast', '-tune', 'zerolatency', 
            '-rtsp_transport', 'udp', '-f', 'rtsp', push_url  
        ]
        try:
            self.process = subprocess.Popen(self.cmd, stdin=subprocess.PIPE, bufsize=0)
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
                self.process.stdin.write(frame.data)
                self.queue.task_done()
            except queue.Empty: continue
            except Exception as e: logging.error(f"Stream Error: {e}")

    def write(self, frame):
        if not self.running: return
        frame_out = cv2.resize(frame, (self.width, self.height)) if (frame.shape[1] != self.width) else frame
        try: self.queue.put_nowait(frame_out)
        except queue.Full: pass

    def stop(self):
        self.running = False
        if hasattr(self, 'thread'): self.thread.join()
        if hasattr(self, 'process') and self.process.stdin:
            self.process.stdin.close(); self.process.wait()

class GStreamerCamera:
    # (Giữ nguyên class Camera)
    def __init__(self, source):
        gst_pipeline = (
            f"rtspsrc location={source} latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! " 
            "queue max-size-buffers=1 leaky=downstream ! videoconvert ! video/x-raw, format=(string)BGR ! " 
            "appsink sync=false drop=true max-buffers=1" 
        )
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(source)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ret, self.latest_frame, self.running = False, None, True
        self.lock = threading.Lock()
        threading.Thread(target=self._update, daemon=True).start()

    def _update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock: self.ret, self.latest_frame = ret, frame
            else: time.sleep(0.005)

    def read(self):
        with self.lock: return self.ret, (self.latest_frame.copy() if self.latest_frame is not None else None)

    def stop(self):
        self.running = False; self.cap.release()

class SiyiGimbalController:
    """
    Class điều khiển Gimbal SIYI - Có Thread lắng nghe phản hồi UDP.
    """
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1) 
        self.seq = 0
        self.last_sent_time = 0
        
        # Biến lưu trữ góc Gimbal hiện tại (Độ)
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.running = True

        # Start luồng lắng nghe
        self.listen_thread = threading.Thread(target=self._listen_worker, daemon=True, name="GimbalListener")
        self.listen_thread.start()
        
        logging.info(f"Gimbal Controller initialized at {ip}:{port}")

    def _append_crc16(self, data):
        crc = 0
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
        return data + struct.pack('<H', crc)

    def _listen_worker(self):
        """Worker đọc dữ liệu trả về từ Gimbal"""
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                if len(data) > 0:
                    self._parse_packet(data)
            except socket.timeout:
                continue
            except Exception as e:
                logging.warning(f"Gimbal Recv Error: {e}")

    def _parse_packet(self, data):
        if len(data) < 10: return
        if data[0] != 0x55 or data[1] != 0x66: return

        cmd_id = data[6]
        # 0x0D: ATTITUDE DATA
        if cmd_id == 0x0D:
            try:
                yaw_raw, pitch_raw, roll_raw = struct.unpack('<hhh', data[8:14])
                self.current_yaw = yaw_raw / 10.0
                self.current_pitch = pitch_raw / 10.0
                self.current_roll = roll_raw / 10.0
            except Exception: pass

    def request_attitude(self):
        """Gửi lệnh hỏi góc (0x0D)"""
        self.seq += 1
        msg_head = b'\x55\x66\x01\x00\x00' + struct.pack('<H', self.seq) + b'\x0D'
        try:
            self.sock.sendto(self._append_crc16(msg_head), (self.ip, self.port))
        except: pass

    def get_attitude(self):
        return self.current_roll, self.current_pitch, self.current_yaw

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
            self.sock.sendto(self._append_crc16(full_msg), (self.ip, self.port))
        except Exception as e: logging.warning(f"Gimbal Send Error: {e}")

    def center(self):
        self.seq += 1
        payload = b'\x01'
        msg_head = b'\x55\x66\x01' + struct.pack('<H', len(payload)) + struct.pack('<H', self.seq) + b'\x08'
        self.sock.sendto(self._append_crc16(msg_head + payload), (self.ip, self.port))

    def set_angle(self, yaw, pitch):
        self.seq += 1
        yaw_val = int(yaw * 10)
        pitch_val = int(pitch * 10)
        payload = struct.pack('<hh', yaw_val, pitch_val)
        msg_head = b'\x55\x66\x01' + struct.pack('<H', len(payload)) + struct.pack('<H', self.seq) + b'\x0E'
        full_msg = msg_head + payload
        try:
            self.sock.sendto(self._append_crc16(full_msg), (self.ip, self.port))
            logging.info(f"Set Angle: Y={yaw} P={pitch}")
        except: pass

    def stop(self):
        self.running = False
        self.rotate(0, 0)
        self.sock.close()

# ==========================================
# --- 5. MAIN LOGIC ---
# ==========================================

def main():
    setup_logging()
    cfg = Config("config.json")
    
    logging.info(">>> SYSTEM STARTING (TRACKING + MATRIX MATH)...")

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
    
    # --- [ĐÃ SỬA LẠI] KHỞI TẠO GÓC NHÌN -90 ĐỘ ---
    logging.info("Initializing Gimbal to -90 degrees...")
    for _ in range(3):
        gimbal.set_angle(0, -90) 
        time.sleep(0.05)
    # ---------------------------------------------

    logger = CSVDataLogger()
    logger.start()

    try:
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        aruco_params = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, aruco_params)
    except AttributeError:
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        aruco_params = aruco.DetectorParameters_create()
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

            # 1. Yêu cầu cập nhật góc Gimbal
            gimbal.request_attitude()

            # 2. Đọc Camera
            ret, frame = camera.read()
            if not ret or frame is None:
                time.sleep(0.001)
                continue

            # 3. Detect ArUco
            algo_start = time.perf_counter()
            detect_counter += 1
            should_detect = (detect_counter % (sys_conf['skip_frame'] + 1) == 0)
            
            if should_detect:
                scale = sys_conf['detect_scale']
                small_frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
                gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
                
                if detector: corners, ids, _ = detector.detectMarkers(gray)
                else: corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
                
                if ids is not None and len(ids) > 0:
                    corners = tuple((c / scale).astype(np.float32) for c in corners)
                    last_corners, last_ids = corners, ids
                else:
                    last_ids = None
            else:
                corners, ids = last_corners, last_ids

            # 4. Tính toán Pose
            body_x, body_y, body_z = 0, 0, 0
            marker_yaw = 0
            cur_roll, cur_pitch, cur_yaw = gimbal.get_attitude()

            if last_ids is not None and last_corners is not None:
                try:
                    marker_points = np.array([
                        [-trk_conf['marker_size']/2, trk_conf['marker_size']/2, 0],
                        [trk_conf['marker_size']/2, trk_conf['marker_size']/2, 0],
                        [trk_conf['marker_size']/2, -trk_conf['marker_size']/2, 0],
                        [-trk_conf['marker_size']/2, -trk_conf['marker_size']/2, 0]
                    ], dtype=np.float32)
                    
                    _, rvec, tvec = cv2.solvePnP(marker_points, last_corners[0], cam_mtx, dist_coeffs, False, cv2.SOLVEPNP_IPPE_SQUARE)
                    
                    # NHÂN MA TRẬN ĐỂ RA TOẠ ĐỘ BODY FRAME
                    body_x, body_y, body_z = get_body_position(tvec.reshape(3), cur_roll, cur_pitch, cur_yaw)
                    
                    rmat, _ = cv2.Rodrigues(rvec)
                    marker_yaw = math.degrees(math.atan2(rmat[1,0], rmat[0,0]))

                    cv2.drawFrameAxes(frame, cam_mtx, dist_coeffs, rvec, tvec, 0.05)
                    aruco.drawDetectedMarkers(frame, last_corners, last_ids)
                    
                    # PID Tracking
                    c = last_corners[0][0]
                    curr_cx, curr_cy = int((c[0][0]+c[2][0])/2), int((c[0][1]+c[2][1])/2)
                    err_x, err_y = curr_cx - center_x, curr_cy - center_y
                    
                    yaw_out = int(err_x * trk_conf['kp_yaw']) if abs(err_x) > trk_conf['deadzone'] else 0
                    pitch_out = int(-err_y * trk_conf['kp_pitch']) if abs(err_y) > trk_conf['deadzone'] else 0

                    if trk_conf['active']: gimbal.rotate(yaw_out, pitch_out)
                    
                    info = f"BX:{body_x:.2f} BY:{body_y:.2f} BZ:{body_z:.2f} P:{cur_pitch:.1f}"
                    cv2.putText(frame, info, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    cv2.line(frame, (center_x, center_y), (curr_cx, curr_cy), (0, 0, 255), 2)
                
                except Exception as e:
                    logging.error(f"Pose Error: {e}")
            else:
                if trk_conf['active']: gimbal.rotate(0, 0)

            algo_time_ms = (time.perf_counter() - algo_start) * 1000

            # 5. Output
            cv2.drawMarker(frame, (center_x, center_y), (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
            cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            pusher.write(frame)

            elapsed = time.perf_counter() - loop_start
            wait_time = frame_time_ms - elapsed
            if wait_time > 0: time.sleep(wait_time)

            frame_count += 1
            if frame_count >= 10:
                fps = frame_count / (time.perf_counter() - start_time_global)
                start_time_global = time.perf_counter(); frame_count = 0
                
                log_payload = [
                    datetime.now().strftime("%H:%M:%S.%f")[:-3],
                    f"{algo_time_ms:.2f}", f"{elapsed*1000:.2f}", f"{fps:.2f}",
                    f"{body_x:.3f}", f"{body_y:.3f}", f"{body_z:.3f}", f"{marker_yaw:.2f}",
                    f"{cur_roll:.1f}", f"{cur_pitch:.1f}", f"{cur_yaw:.1f}"
                ]
                logger.log(log_payload)

    except KeyboardInterrupt: logging.info("Stop.")
    except Exception as e: logging.critical(f"Crash: {e}")
    finally:
        camera.stop(); pusher.stop(); gimbal.stop(); logger.stop()

if __name__ == "__main__":
    main()