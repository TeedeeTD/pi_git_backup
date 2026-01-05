import sys
import time
import threading
import socket
import struct
import cv2
import cv2.aruco as aruco
import collections
import csv
from datetime import datetime
import queue
import numpy as np
import math
import logging

# ==========================================
# --- CẤU HÌNH LOGGING & HỆ THỐNG ---
# ==========================================
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

DETECT_SCALE = 0.7  
SKIP_FRAME = 1      

# URL Server đích (MediaMTX)
RTSP_PUSH_URL = "rtsp://localhost:8554/siyi_aruco"
# URL Camera nguồn (SIYI)
CAMERA_URL = "rtsp://127.0.0.1:8554/my_camera" 

GPS_PORT = 5555
STREAM_W, STREAM_H = 1280, 720 
FIXED_FPS = 30   
FRAME_TIME_MS = 1.0 / FIXED_FPS 

# --- CẤU HÌNH TRACKING ---
SIYI_IP = "192.168.168.14"
SIYI_PORT = 37260
TRACKING_ACTIVE = True
KP_YAW, KP_PITCH, DEADZONE = 0.15, 0.25, 15     

# --- CAMERA MATRIX ---
CAMERA_MATRIX = np.array([[717.14, 0, 664.29], [0, 717.88, 354.24], [0, 0, 1]], dtype=np.float32)
DIST_COEFFS = np.array([[-0.0764, 0.0742, -0.0013, 0.0019, -0.0176]], dtype=np.float32)
MARKER_SIZE = 0.1 

# ==========================================
# --- HELPER FUNCTIONS ---
# ==========================================
def my_estimatePoseSingleMarkers(corners, marker_size, mtx, dist):
    marker_points = np.array([
        [-marker_size/2, marker_size/2, 0], [marker_size/2, marker_size/2, 0],
        [marker_size/2, -marker_size/2, 0], [-marker_size/2, -marker_size/2, 0]
    ], dtype=np.float32)
    rvecs, tvecs = [], []
    for c in corners:
        _, r, t = cv2.solvePnP(marker_points, c, mtx, dist, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(r.reshape(1, 3))
        tvecs.append(t.reshape(1, 3))
    return rvecs, tvecs

def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0,0]*R[0,0] + R[1,0]*R[1,0])
    return np.array([math.atan2(R[2,1], R[2,2]), math.atan2(-R[2,0], sy), math.atan2(R[1,0], R[0,0])])

# ==========================================
# --- CLASSES ---
# ==========================================

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
                crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
                crc &= 0xFFFF
        return data + struct.pack('<H', crc)

    def rotate(self, yaw, pitch):
        # Rate limit gửi lệnh để tránh spam gimbal
        if time.time() - self.last_sent < 0.05: 
            return 
        self.last_sent = time.time()
        self.seq += 1
        
        # Clamp giá trị -100 đến 100
        y_val = max(min(int(yaw), 100), -100)
        p_val = max(min(int(pitch), 100), -100)
        
        payload = struct.pack('<bb', y_val, p_val) 
        msg = b'\x55\x66\x01' + struct.pack('<H', len(payload)) + struct.pack('<H', self.seq) + b'\x07' + payload
        
        try:
            full_msg = self._append_crc16(msg)
            # Chỉ lấy 2 byte cuối của CRC tính được append vào
            self.sock.sendto(full_msg, (self.ip, self.port))
        except Exception as e:
            logging.error(f"Gimbal Error: {e}")

    def center(self):
        self.seq += 1
        payload = b'\x01'
        msg = b'\x55\x66\x01' + struct.pack('<H', len(payload)) + struct.pack('<H', self.seq) + b'\x08' + payload
        try:
            self.sock.sendto(self._append_crc16(msg), (self.ip, self.port))
            logging.info("Gimbal Centered")
        except: pass

    def stop(self):
        self.rotate(0, 0)

class CameraStream:
    def __init__(self, src):
        # Pipeline tối ưu cho decode RTSP (Low latency)
        gst_pipeline = (
            f"rtspsrc location={src} latency=0 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! "
            "queue ! " 
            "videoconvert ! video/x-raw, format=(string)BGR ! "
            "appsink sync=false drop=true max-buffers=1"
        )
        
        logging.info(f"Opening Camera Pipeline:\n{gst_pipeline}")
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            logging.warning("GStreamer Failed. Switching to Standard V4L2/FFmpeg backend...")
            self.cap = cv2.VideoCapture(src)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        else:
            logging.info("✅ Camera Connected (GStreamer Mode)")

        self.ret = False
        self.frame = None
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
            else:
                time.sleep(0.01) # Sleep nhẹ để giảm CPU load khi mất tín hiệu

    def read(self):
        with self.lock:
            return self.ret, self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        self.t.join()
        self.cap.release()

class StreamPusher:
    def __init__(self, dest_url, w, h, fps, bitrate_k=2000):
        self.w, self.h = w, h
        self.running = False
        
        # Encoder Pipeline cho Raspberry Pi (Hardware Accelerated)
        # Nếu chạy trên PC thường (không có v4l2h264enc), hãy đổi 'v4l2h264enc' thành 'x264enc'
        encoder_elem = f"v4l2h264enc extra-controls=\"controls,video_bitrate={bitrate_k}000\" output-io-mode=4 capture-io-mode=4"
        
        self.gst_pipeline = (
            f"appsrc ! "
            f"video/x-raw, format=BGR, width={w}, height={h}, framerate={fps}/1 ! "
            f"queue max-size-buffers=1 leaky=downstream ! "
            f"videoconvert ! "
            f"video/x-raw, format=I420 ! "
            f"{encoder_elem} ! "
            f"h264parse ! "
            f"rtspclientsink location={dest_url} protocols=tcp latency=0"
        )

        logging.info(f"Init StreamPusher Pipeline:\n{self.gst_pipeline}")

        try:
            self.out = cv2.VideoWriter(self.gst_pipeline, cv2.CAP_GSTREAMER, 0, fps, (w, h), True)
            if not self.out.isOpened():
                logging.error("❌ StreamPusher Failed to Open Pipeline!")
            else:
                self.running = True
                logging.info("✅ StreamPusher Ready")
        except Exception as e:
            logging.error(f"StreamPusher Exception: {e}")

    def write(self, frame):
        if not self.running: return
        # Resize an toàn
        if frame.shape[1] != self.w or frame.shape[0] != self.h:
            frame = cv2.resize(frame, (self.w, self.h))
        self.out.write(frame)

    def stop(self):
        self.running = False
        if hasattr(self, 'out') and self.out:
            self.out.release()

# ==========================================
# --- MAIN PROGRAM ---
# ==========================================
running_global = True
current_lat, current_lon = 0.0, 0.0

def gps_thread():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", GPS_PORT))
    s.settimeout(1.0)
    while running_global:
        try:
            d, _ = s.recvfrom(1024)
            _, _, lat, lon, _ = struct.unpack('qi3d', d)
            global current_lat, current_lon
            current_lat, current_lon = lat, lon
        except socket.timeout:
            pass
        except Exception:
            pass

def main():
    global running_global
    print(f">>> PI TRACKING V5 (The Real Deal)...")
    
    # Init Camera
    cam = CameraStream(CAMERA_URL)
    time.sleep(2.0) # Đợi camera warm up
    
    if not cam.ret:
        logging.error("❌ Camera not ready! Exiting...")
        cam.stop()
        return
    
    # Init Stream Pusher (GStreamer Native)
    pusher = StreamPusher(RTSP_PUSH_URL, STREAM_W, STREAM_H, FIXED_FPS)
    
    # Init Gimbal
    gimbal = SiyiGimbal(SIYI_IP, SIYI_PORT)
    gimbal.center()
    
    # Init ArUco
    try:
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        params = aruco.DetectorParameters_create()
    except AttributeError:
        # Hỗ trợ OpenCV mới
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        params = aruco.DetectorParameters()
    
    params.adaptiveThreshWinSizeStep = 10
    detector = aruco.ArucoDetector(aruco_dict, params) if hasattr(aruco, "ArucoDetector") else None

    # CSV Logging
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"track_log_{timestamp_str}.csv"
    
    try:
        csv_file = open(csv_filename, mode='w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["Timestamp", "Algo_ms", "Loop_ms", "FPS", "Pos_X", "Pos_Y", "Pos_Z", "Roll", "Pitch", "Yaw"])
        logging.info(f"Logging to {csv_filename}")
    except Exception as e:
        logging.error(f"Cannot create CSV file: {e}")
        csv_writer = None

    # Variables
    fc = 0
    st = time.perf_counter()
    fps = 0
    center = (STREAM_W//2, STREAM_H//2)
    
    # Biến lưu trạng thái tracking
    last_corners, last_ids = None, None
    detect_count = 0 
    
    logging.info(">>> SYSTEM LIVE! Press Ctrl+C to stop.")

    try:
        while True:
            loop_start = time.perf_counter()
            
            # 1. Đọc Frame
            ret, frame = cam.read()
            if not ret or frame is None:
                time.sleep(0.005)
                continue

            # 2. Xử lý ArUco (có Skip Frame)
            ts_algo_start = time.perf_counter()
            detect_count += 1
            should_detect = (detect_count % (SKIP_FRAME + 1) == 0)

            if should_detect:
                # Resize để detect nhanh hơn
                sw, sh = int(STREAM_W * DETECT_SCALE), int(STREAM_H * DETECT_SCALE)
                small = cv2.resize(frame, (sw, sh), interpolation=cv2.INTER_NEAREST)
                gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
                
                if detector:
                    corners, ids, _ = detector.detectMarkers(gray)
                else:
                    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)
                
                if ids is not None and len(ids) > 0:
                    # Scale lại tọa độ về size gốc
                    corners = tuple((c / DETECT_SCALE).astype(np.float32) for c in corners)
                    last_corners, last_ids = corners, ids
                else:
                    last_ids = None # Mất dấu
            else:
                corners, ids = last_corners, last_ids

            # 3. Tính toán Pose & Control
            px, py, pz, r, p, y = 0,0,0,0,0,0
            is_tracking = False
            ly, lp = 0, 0

            if last_ids is not None and last_corners is not None:
                is_tracking = True
                try:
                    rvecs, tvecs = my_estimatePoseSingleMarkers(last_corners, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS)
                    px, py, pz = tvecs[0][0]
                    
                    # Vẽ trục tọa độ 3D
                    cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, rvecs[0], tvecs[0], 0.05)
                    aruco.drawDetectedMarkers(frame, last_corners, last_ids)
                    
                    # Tính góc Euler
                    rmat, _ = cv2.Rodrigues(rvecs[0])
                    euler = rotationMatrixToEulerAngles(rmat)
                    r, p, y = [math.degrees(x) for x in euler]
                except Exception as e:
                    # logging.warning(f"Pose Error: {e}")
                    pass

                # Logic Gimbal (P-Controller đơn giản)
                # Tính tâm của Marker đầu tiên
                c = last_corners[0][0]
                cx, cy = int((c[0][0]+c[2][0])/2), int((c[0][1]+c[2][1])/2)
                
                # Sai số so với tâm khung hình
                ex, ey = cx - center[0], cy - center[1]
                
                if abs(ex) > DEADZONE: ly = int(ex * KP_YAW)
                if abs(ey) > DEADZONE: lp = int(-ey * KP_PITCH)
                
                cv2.line(frame, center, (cx, cy), (0,0,255), 2)

            if TRACKING_ACTIVE:
                gimbal.rotate(ly, lp)
            
            algo_ms = (time.perf_counter() - ts_algo_start) * 1000

            # 4. Vẽ OSD & Push Stream
            cv2.drawMarker(frame, center, (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
            cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            if is_tracking:
                cv2.putText(frame, f"POS: {px:.2f}, {py:.2f}, {pz:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            pusher.write(frame)

            # 5. Timing & Logging
            loop_elapsed = time.perf_counter() - loop_start
            loop_ms = loop_elapsed * 1000
            wait_time = FRAME_TIME_MS - loop_elapsed
            
            if wait_time > 0:
                time.sleep(wait_time)
            
            fc += 1
            if fc >= 15: # Cập nhật FPS mỗi 15 frames
                now = time.perf_counter()
                fps = fc / (now - st)
                st = now
                fc = 0
                
                # Ghi CSV
                if csv_writer:
                    try:
                        csv_writer.writerow([
                            datetime.now().strftime("%H:%M:%S.%f")[:-3], 
                            f"{algo_ms:.2f}", f"{loop_ms:.2f}", f"{fps:.2f}", 
                            f"{px:.3f}", f"{py:.3f}", f"{pz:.3f}", 
                            f"{r:.2f}", f"{p:.2f}", f"{y:.2f}"
                        ])
                    except: pass

    except KeyboardInterrupt:
        logging.info("Interrupted by User")
    finally:
        running_global = False
        cam.stop()
        pusher.stop()
        gimbal.stop()
        if csv_file: csv_file.close()
        logging.info("System Shutdown Cleanly.")

if __name__ == "__main__":
    t = threading.Thread(target=gps_thread, daemon=True)
    t.start()
    main()
