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
    """
    Class quản lý cấu hình hệ thống.
    Nhiệm vụ: Load file config.json, nếu không có thì dùng giá trị mặc định (DEFAULTS).
    Giúp tách biệt phần code logic và phần tham số điều chỉnh.
    """
    
    # Cấu hình mặc định - Sẽ được dùng nếu không tìm thấy file config.json
    DEFAULTS = {
        "system": {
            "detect_scale": 0.7,   # [0.1 - 1.0] Giảm kích thước ảnh đầu vào khi detect ArUco. 
                                   # 0.7 nghĩa là giảm còn 70%. Giúp tăng FPS đáng kể nhưng giảm khả năng detect xa.
            "skip_frame": 1,       # [Integer] Số frame bỏ qua không detect. 
                                   # 1 = Detect 1 frame, bỏ 1 frame (Detect ở 15FPS nếu cam 30FPS). Giảm tải CPU.
            "input_width": 1280,   # Độ phân giải gốc của Camera đầu vào
            "input_height": 720,
            "stream_scale": 0.5,   # [0.1 - 1.0] Tỷ lệ resize ảnh khi stream qua RTSP/Wifi.
                                   # 0.5 = 640x360. Giúp giảm băng thông mạng và độ trễ truyền hình ảnh.
            "target_fps": 30,      # FPS mục tiêu của vòng lặp chính. Dùng để tính toán thời gian ngủ (sleep) giữ nhịp.
            "gps_port": 5555       # Port UDP nhận dữ liệu GPS (nếu có module khác gửi sang).
        },
        "connection": {
            "rtsp_push_url": "rtsp://localhost:8554/siyi_aruco", # Địa chỉ server RTSP để đẩy luồng video đã xử lý lên.
            "camera_source": "rtsp://127.0.0.1:8554/my_camera",  # Nguồn video đầu vào (RTSP Camera SIYI hoặc Webcam).
            "siyi_ip": "192.168.168.14", # IP mặc định của Camera SIYI A8 mini/ZR10.
            "siyi_port": 37260           # Port UDP điều khiển Gimbal của SIYI (SDK Port).
        },
        "tracking": {
            "active": True,        # [True/False] Cho phép gửi lệnh điều khiển gimbal hay không.
            "kp_yaw": 0.15,        # [Hệ số P - Proportional] Độ nhạy trục xoay ngang. 
                                   # Lớn = Quay nhanh về tâm nhưng dễ bị rung (overshoot). Nhỏ = Quay mượt nhưng chậm.
            "kp_pitch": 0.25,      # [Hệ số P] Độ nhạy trục ngẩng/cụp. Thường cần lớn hơn Yaw do trọng lực/cơ khí.
            "deadzone": 15,        # [Pixel] Vùng chết. Nếu tâm ArUco lệch ít hơn 15px so với tâm ảnh thì KHÔNG quay.
                                   # Giúp gimbal không bị rung lắc liên tục khi ArUco đã ở gần giữa.
            "marker_size": 0.156     # [Mét] Kích thước thực tế của in mã ArUco (tính cạnh đen ngoài cùng).
                                   # QUAN TRỌNG: Sai số này sẽ dẫn đến tính sai khoảng cách X, Y, Z.
        },
        "camera_matrix": {
            # Ma trận nội tham số Camera (Intrinsics) - Cần Calibrate để có độ chính xác cao nhất
            "matrix": [[717.14, 0, 664.29], [0, 717.88, 354.24], [0, 0, 1]],
            "dist_coeffs": [[-0.0764, 0.0742, -0.0013, 0.0019, -0.0176]] # Hệ số méo kính (Distortion)
        }
    }

    def __init__(self, config_path="config.json"):
        """
        Khởi tạo Config.
        Input: config_path (đường dẫn file json).
        Logic: Thử mở file, nếu lỗi thì giữ nguyên DEFAULTS.
        """
        self.data = self.DEFAULTS
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r') as f:
                    file_data = json.load(f)
                    self.data.update(file_data) # Ghi đè cấu hình từ file lên default
                logging.info(f"Loaded configuration from {config_path}")
            except Exception as e:
                logging.error(f"Failed to load config file: {e}. Using defaults.")
        else:
            logging.warning("Config file not found. Using default values.")

    def get(self, section, key):
        """Hàm helper để lấy giá trị cấu hình an toàn, tránh lỗi KeyError."""
        return self.data.get(section, {}).get(key)

# ==========================================
# --- 2. LOGGING SETUP ---
# ==========================================
def setup_logging():
    """
    Thiết lập hệ thống ghi log.
    - Console: In ra màn hình để debug trực tiếp.
    - File: Ghi lỗi (Warning/Error) vào file system_error.log để tra cứu sau.
    """
    log_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(threadName)s - %(message)s')
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)

    # Console Handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(log_formatter)
    root_logger.addHandler(console_handler)

    # File Handler
    file_handler = logging.FileHandler('system_error.log')
    file_handler.setLevel(logging.WARNING)
    file_handler.setFormatter(log_formatter)
    root_logger.addHandler(file_handler)

# ==========================================
# --- 3. HELPER CLASSES ---
# ==========================================

class CSVDataLogger(threading.Thread):
    """
    Class ghi log dữ liệu bay (Data Logging) chạy trên luồng riêng (Thread).
    Mục đích: Việc ghi file I/O rất chậm, nếu để ở Main Loop sẽ làm tụt FPS.
    Giải pháp: Main Loop đẩy dữ liệu vào Queue, Class này lấy ra ghi từ từ.
    """
    def __init__(self, prefix="track_log"):
        super().__init__(daemon=True, name="CSVLoggerThread")
        self.queue = queue.Queue()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"{prefix}_{timestamp}.csv"
        # Header file CSV
        # --- CẬP NHẬT HEADER CHO PHÙ HỢP VỚI HỆ TỌA ĐỘ BODY FRAME ---
        self.header = ["Timestamp", "Algo_ms", "Loop_ms", "FPS", "Body_X", "Body_Y", "Alt_Z", "Marker_Yaw"]
        self.running = True
        
        # Tạo file và ghi header ngay lập tức
        try:
            with open(self.filename, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.header)
            logging.info(f"CSV Logger initialized: {self.filename}")
        except Exception as e:
            logging.error(f"Failed to create CSV file: {e}")

    def log(self, data):
        """
        Hàm được gọi từ Main Loop.
        Input: list dữ liệu (data).
        Hành động: Đẩy ngay vào Queue (Non-blocking) rồi return để Main Loop chạy tiếp.
        """
        if self.running:
            self.queue.put(data)

    def run(self):
        """
        Hàm chạy ngầm của Thread.
        Hành động: Liên tục kiểm tra Queue, nếu có dữ liệu thì ghi xuống file.
        """
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
    """
    Class đẩy luồng video ra ngoài qua RTSP bằng FFmpeg.
    [TỐI ƯU HÓA]: Việc resize ảnh được thực hiện trong Thread Worker, không làm phiền Main Loop.
    """
    def __init__(self, push_url, width, height, fps):
        self.width = width
        self.height = height
        self.target_size = (width, height) # Lưu kích thước đích
        self.running = True
        self.queue = queue.Queue(maxsize=2) # Queue nhỏ (2 frames) để giảm độ trễ (latency)
        
        # --- Cấu hình lệnh FFmpeg (đã tối ưu Zero Latency) ---
        self.cmd = [
            'ffmpeg', '-y', 
            '-f', 'rawvideo',       # Đầu vào: Raw
            '-vcodec', 'rawvideo', 
            '-pix_fmt', 'bgr24',    # Định dạng: BGR (OpenCV Default) - Tránh convert màu ở Python
            '-s', f'{width}x{height}', 
            '-r', str(fps), 
            '-i', '-',              # Input: STDIN Pipe
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
            # Khởi tạo subprocess và mở Pipe 'stdin' để ghi dữ liệu
            self.process = subprocess.Popen(self.cmd, stdin=subprocess.PIPE, bufsize=0)
            logging.info(f"FFmpeg started streaming to {push_url}")
        except Exception as e:
            logging.critical(f"Failed to start FFmpeg: {e}")
            self.running = False
            return

        # Tạo thread riêng để ghi dữ liệu vào Pipe tránh block chương trình chính
        self.thread = threading.Thread(target=self._worker, daemon=True, name="StreamerThread")
        self.thread.start()

    def _worker(self):
        """
        Thread worker: 
        1. Lấy frame gốc từ Queue.
        2. Resize frame (Tốn CPU, nên làm ở đây thay vì Main Loop).
        3. Nhét vào họng (STDIN) của FFmpeg.
        """
        while self.running:
            try:
                frame = self.queue.get(timeout=1)
                
                # [OPTIMIZATION] Thực hiện resize tại đây để giải phóng Main Loop
                if frame.shape[1] != self.width or frame.shape[0] != self.height:
                    # Dùng INTER_LINEAR hoặc INTER_NEAREST để nhanh hơn
                    frame_out = cv2.resize(frame, self.target_size, interpolation=cv2.INTER_LINEAR)
                else:
                    frame_out = frame

                # Ghi byte vào pipe. Dùng tobytes() nhanh hơn .data
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
        """
        Main Loop gọi hàm này để gửi frame đi stream.
        Logic: Chỉ việc tống vào Queue. Cực nhanh.
        """
        if not self.running: return

        try:
            # put_nowait: Không chờ, nếu đầy thì văng Exception Full -> Drop frame để giữ realtime
            self.queue.put_nowait(frame) 
        except queue.Full:
            pass # Drop frame gracefully

    def stop(self):
        self.running = False
        if hasattr(self, 'thread'): self.thread.join()
        if hasattr(self, 'process') and self.process.stdin:
            self.process.stdin.close()
            self.process.wait()

class GStreamerCamera:
    """
    Class đọc Camera sử dụng GStreamer Pipeline (Tối ưu cho độ trễ thấp).
    Hoạt động trên thread riêng để buffer camera luôn được làm mới.
    """
    def __init__(self, source):
        # Pipeline tối ưu: Lấy stream -> decode -> vứt vào appsink
        gst_pipeline = (
            f"rtspsrc location={source} latency=0 ! " 
            "rtph264depay ! h264parse ! avdec_h264 ! " 
            "queue max-size-buffers=1 leaky=downstream ! " # Chỉ giữ 1 frame mới nhất
            "videoconvert ! video/x-raw, format=(string)BGR ! " 
            "appsink sync=false drop=true max-buffers=1" 
        )
        
        logging.info(f"Connecting to Camera: {source}")
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        # Fallback nếu GStreamer lỗi
        if not self.cap.isOpened():
            logging.warning("GStreamer failed. Fallback to standard capture.")
            self.cap = cv2.VideoCapture(source)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.ret = False
        self.latest_frame = None
        self.running = True
        self.lock = threading.Lock() # Lock để tránh Race Condition khi đọc/ghi frame
        
        self.thread = threading.Thread(target=self._update, daemon=True, name="CameraThread")
        self.thread.start()

    def _update(self):
        """Thread liên tục đọc frame từ driver camera."""
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock: # Khóa biến lại trước khi ghi đè
                    self.ret = ret
                    self.latest_frame = frame
            else:
                time.sleep(0.005)

    def read(self):
        """Main Loop gọi hàm này để lấy frame mới nhất."""
        with self.lock: # Khóa biến lại trước khi copy
            if self.latest_frame is not None:
                return self.ret, self.latest_frame.copy() # Trả về bản sao an toàn
            return False, None

    def stop(self):
        self.running = False
        self.thread.join()
        self.cap.release()

class SiyiGimbalController:
    """
    Class điều khiển Gimbal SIYI qua giao thức UDP SDK.
    Chịu trách nhiệm đóng gói packet, tính CRC16 và gửi lệnh.
    """
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.last_sent_time = 0
        logging.info(f"Gimbal Controller initialized at {ip}:{port}")

    def _append_crc16(self, data):
        """Tính toán Checksum CRC16 theo tài liệu SDK của SIYI."""
        crc = 0
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
        return data + struct.pack('<H', crc)

    def rotate(self, yaw_speed, pitch_speed):
        """
        Gửi lệnh xoay (Speed Control).
        Input: yaw_speed [-100, 100], pitch_speed [-100, 100].
        """
        # Rate Limiting: Giới hạn tốc độ gửi lệnh (max 20Hz) để tránh tràn bộ đệm Gimbal
        if time.time() - self.last_sent_time < 0.05: 
            return

        self.last_sent_time = time.time()
        self.seq += 1
        
        # Clamp: Đảm bảo giá trị nằm trong khoảng cho phép
        y_val = max(min(int(yaw_speed), 100), -100)
        p_val = max(min(int(pitch_speed), 100), -100)
        
        payload = struct.pack('<bb', y_val, p_val)
        # 0x07 là Command ID cho chức năng Rotate
        msg_head = b'\x55\x66\x01' + struct.pack('<H', len(payload)) + struct.pack('<H', self.seq) + b'\x07'
        full_msg = msg_head + payload
        
        try:
            packet = self._append_crc16(full_msg)
            final_packet = full_msg + packet[-2:] 
            self.sock.sendto(final_packet, (self.ip, self.port))
        except Exception as e:
            logging.warning(f"Gimbal Send Error: {e}")

    def center(self):
        """Gửi lệnh yêu cầu Gimbal quay về vị trí trung tâm (0,0)."""
        self.seq += 1
        payload = b'\x01'
        # 0x08 là Command ID cho chức năng Center
        msg_head = b'\x55\x66\x01' + struct.pack('<H', len(payload)) + struct.pack('<H', self.seq) + b'\x08'
        full_msg = msg_head + payload
        try:
            packet = self._append_crc16(full_msg)
            self.sock.sendto(full_msg + packet[-2:], (self.ip, self.port))
            logging.info("Sent Center Command")
        except Exception as e:
            logging.error(f"Gimbal Center Error: {e}")

    # --- HÀM SET GÓC TUYỆT ĐỐI ---
    def set_angle(self, yaw, pitch):
        """
        Gửi lệnh 0x0E: Set góc tuyệt đối.
        yaw: Góc xoay ngang (độ)
        pitch: Góc ngẩng/cúi (độ). Ví dụ -90 là cúi thẳng xuống.
        """
        self.seq += 1
        
        # SIYI dùng đơn vị 0.1 độ (int16)
        yaw_val = int(yaw * 10)
        pitch_val = int(pitch * 10)
        
        payload = struct.pack('<hh', yaw_val, pitch_val)
        
        # 0x0E là Command ID cho chức năng Set Angle
        msg_head = b'\x55\x66\x01' + struct.pack('<H', len(payload)) + struct.pack('<H', self.seq) + b'\x0E'
        full_msg = msg_head + payload
        
        try:
            packet = self._append_crc16(full_msg)
            self.sock.sendto(packet, (self.ip, self.port))
            # Giảm bớt log để tránh spam console khi gọi định kỳ
            # logging.info(f"Sent Angle Command: Yaw={yaw}, Pitch={pitch}") 
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
    
    logging.info(">>> SYSTEM STARTING (OPTIMIZED & FIXED -90 DEGREE MODE)...")

    # Load Params từ Config object
    sys_conf = cfg.data['system']
    conn_conf = cfg.data['connection']
    trk_conf = cfg.data['tracking']
    cam_mtx = np.array(cfg.data['camera_matrix']['matrix'], dtype=np.float32)
    dist_coeffs = np.array(cfg.data['camera_matrix']['dist_coeffs'], dtype=np.float32)

    # Tính toán kích thước stream đầu ra
    stream_w = int(sys_conf['input_width'] * sys_conf['stream_scale'])
    stream_h = int(sys_conf['input_height'] * sys_conf['stream_scale'])
    frame_time_ms = 1.0 / sys_conf['target_fps']

    # --- KHỞI TẠO MODULE ---
    camera = GStreamerCamera(conn_conf['camera_source'])
    time.sleep(2.0) # Đợi camera warm-up
    
    if not camera.ret:
        logging.critical("Camera not ready! Exiting.")
        camera.stop()
        return

    pusher = FFmpegStreamer(conn_conf['rtsp_push_url'], stream_w, stream_h, sys_conf['target_fps'])
    gimbal = SiyiGimbalController(conn_conf['siyi_ip'], conn_conf['siyi_port'])
    
    # --- SETUP GÓC -90 ĐỘ BAN ĐẦU ---
    logging.info("Setting Gimbal to -90 degrees...")
    for _ in range(3):
        gimbal.set_angle(0, -90) 
        time.sleep(0.05)
    
    # Biến theo dõi thời gian để reset gimbal định kỳ
    last_gimbal_reset_time = time.time()
    GIMBAL_RESET_INTERVAL = 5.0 # Reset mỗi 5 giây

    logger = CSVDataLogger()
    logger.start()

    # --- SETUP ARUCO DETECTOR VỚI CORNER REFINEMENT ---
    try:
        # Code cho OpenCV mới (4.7+)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        aruco_params = aruco.DetectorParameters()
        
        # [OPTIMIZATION] Bật thuật toán tinh chỉnh góc (Subpixel) để tăng độ chính xác trục Z
        aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX 
        # aruco_params.cornerRefinementWinSize = 5 # Mặc định là 5, có thể chỉnh nếu cần
        #Chống nhiễu sáng tốt hơn
        aruco_params.adaptiveThreshWinSizeStep = 10
        
        detector = aruco.ArucoDetector(aruco_dict, aruco_params)
    except AttributeError:
        # Fallback cho OpenCV cũ
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        aruco_params = aruco.DetectorParameters_create()
        # Đối với bản cũ, thuộc tính có thể khác, nhưng thường vẫn là cornerRefinementMethod
        try:
            aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        except:
            pass
        detector = None 

    # --- KHỞI TẠO BIẾN VÒNG LẶP ---
    frame_count = 0
    start_time_global = time.perf_counter()
    fps = 0.0
    
    center_x, center_y = sys_conf['input_width'] // 2, sys_conf['input_height'] // 2
    
    # Biến trạng thái Tracking
    last_corners, last_ids = None, None
    detect_counter = 0
    
    logging.info(">>> LOOP STARTED")

    try:
        while True:
            loop_start = time.perf_counter()

            # ---------------------------
            # BƯỚC 1: ĐỌC CAMERA
            # ---------------------------
            ret, frame = camera.read()
            if not ret or frame is None:
                time.sleep(0.001)
                continue

            # ---------------------------
            # BƯỚC 2: PHÁT HIỆN ARUCO
            # ---------------------------
            algo_start = time.perf_counter()
            detect_counter += 1
            # Logic Skip Frame: Chỉ detect 1 lần mỗi (skip_frame + 1) frame
            should_detect = (detect_counter % (sys_conf['skip_frame'] + 1) == 0)
            
            if should_detect:
                scale = sys_conf['detect_scale']
                # Resize ảnh nhỏ để detect nhanh hơn
                small_frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
                gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
                
                if detector:
                    corners, ids, _ = detector.detectMarkers(gray)
                else:
                    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
                
                if ids is not None and len(ids) > 0:
                    # Nhân lại tọa độ với scale để map về ảnh gốc
                    corners = tuple((c / scale).astype(np.float32) for c in corners)
                    last_corners, last_ids = corners, ids
                else:
                    last_ids = None
            else:
                # Dùng lại kết quả cũ nếu đang ở frame bị skip
                corners, ids = last_corners, last_ids

            # ---------------------------
            # BƯỚC 3: TÍNH TOÁN POSE
            # ---------------------------
            body_x, body_y, body_z = 0, 0, 0
            marker_yaw = 0
            is_tracking = False

            if last_ids is not None and last_corners is not None:
                is_tracking = True
                try:
                    # Định nghĩa 4 điểm góc Marker trong không gian 3D
                    marker_points = np.array([
                        [-trk_conf['marker_size']/2, trk_conf['marker_size']/2, 0],
                        [trk_conf['marker_size']/2, trk_conf['marker_size']/2, 0],
                        [trk_conf['marker_size']/2, -trk_conf['marker_size']/2, 0],
                        [-trk_conf['marker_size']/2, -trk_conf['marker_size']/2, 0]
                    ], dtype=np.float32)
                    
                    # Giải bài toán PnP: Tìm vị trí Camera so với Marker
                    # Sử dụng SOLVEPNP_IPPE_SQUARE để tăng tốc độ và độ chính xác cho planar marker
                    _, rvec, tvec = cv2.solvePnP(marker_points, last_corners[0], cam_mtx, dist_coeffs, False, cv2.SOLVEPNP_IPPE_SQUARE)
                    
                    # --- CHUYỂN ĐỔI HỆ TỌA ĐỘ (Cam nhìn xuống) ---
                    body_x = -tvec[1][0]  # Máy bay cần tiến/lùi (Ngược dấu Y camera)
                    body_y = tvec[0][0]   # Máy bay cần sang phải/trái (Trùng X camera)
                    body_z = tvec[2][0]   # Độ cao
                    
                    # Tính góc xoay Yaw của Marker (Heading)
                    rmat, _ = cv2.Rodrigues(rvec)
                    marker_yaw = math.degrees(math.atan2(rmat[1,0], rmat[0,0]))

                    # Visualization
                    cv2.drawFrameAxes(frame, cam_mtx, dist_coeffs, rvec, tvec, 0.05)
                    aruco.drawDetectedMarkers(frame, last_corners, last_ids)
                    
                    # Vẽ đường nối tâm
                    c = last_corners[0][0]
                    curr_cx = int((c[0][0] + c[2][0]) / 2) 
                    curr_cy = int((c[0][1] + c[2][1]) / 2) 
                    cv2.line(frame, (center_x, center_y), (curr_cx, curr_cy), (0, 0, 255), 2)
                    
                    # Hiển thị thông tin Body Frame
                    info_text = f"X:{body_x:.2f}m Y:{body_y:.2f}m H:{body_z:.2f}m Yaw:{marker_yaw:.1f}"
                    cv2.putText(frame, info_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                except Exception as e:
                    logging.error(f"Pose Estimation Error: {e}")
            
            # ---------------------------
            # BƯỚC 4: QUẢN LÝ GIMBAL (WATCHDOG CHECK)
            # ---------------------------
            # Kiểm tra định kỳ để ép Gimbal nhìn xuống đất
            current_time = time.time()
            if current_time - last_gimbal_reset_time > GIMBAL_RESET_INTERVAL:
                # Gửi lệnh set angle (Non-blocking logic)
                gimbal.set_angle(0, -90)
                last_gimbal_reset_time = current_time
                # Không log gì ở đây để tránh spam console
            
            algo_time_ms = (time.perf_counter() - algo_start) * 1000

            # ---------------------------
            # BƯỚC 5: OSD (HIỂN THỊ)
            # ---------------------------
            cv2.drawMarker(frame, (center_x, center_y), (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
            cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # ---------------------------
            # BƯỚC 6: STREAM OUTPUT
            # ---------------------------
            # [OPTIMIZED] Gọi write, việc resize sẽ do Thread khác lo
            pusher.write(frame)

            # ---------------------------
            # BƯỚC 7: LOGGING & FPS CONTROL
            # ---------------------------
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
                    f"{body_x:.3f}",   
                    f"{body_y:.3f}",   
                    f"{body_z:.3f}",   
                    f"{marker_yaw:.2f}" 
                ]
                logger.log(log_payload)

    except KeyboardInterrupt:
        logging.info("User requested stop.")
    except Exception as e:
        logging.critical(f"Main Loop Crash: {e}")
    finally:
        # ---------------------------
        # CLEANUP (DỌN DẸP)
        # ---------------------------
        camera.stop()
        pusher.stop()
        gimbal.stop()
        logger.stop()
        logging.info(">>> SHUTDOWN COMPLETE")

if __name__ == "__main__":
    main()
