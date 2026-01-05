import sys
import time
import threading
import socket
import struct
import cv2
import cv2.aruco as aruco
import subprocess
import csv
from datetime import datetime
import queue
import numpy as np
import math

# ==========================================
# --- CẤU HÌNH HỆ THỐNG ---
# ==========================================
DETECT_SCALE = 0.7       
SKIP_FRAME = 1           

# URL Streaming
RTSP_PUSH_URL = "rtsp://localhost:8554/siyi_aruco"
CAMERA_URL = "rtsp://127.0.0.1:8554/my_camera" 

# Cấu hình kích thước ảnh
INPUT_W, INPUT_H = 1280, 720   
STREAM_SCALE_OUT = 0.5         
STREAM_W = int(INPUT_W * STREAM_SCALE_OUT)
STREAM_H = int(INPUT_H * STREAM_SCALE_OUT)

FIXED_FPS = 30   
FRAME_TIME_MS = 1.0 / FIXED_FPS 
GPS_PORT = 5555

# --- CẤU HÌNH TRACKING ---
SIYI_IP = "192.168.168.14"
SIYI_PORT = 37260
TRACKING_ACTIVE = True
KP_YAW, KP_PITCH, DEADZONE = 0.15, 0.2, 15     

# --- CAMERA MATRIX ---
CAMERA_MATRIX = np.array([[717.14, 0, 664.29], [0, 717.88, 354.24], [0, 0, 1]], dtype=np.float32)
DIST_COEFFS = np.array([[-0.0764, 0.0742, -0.0013, 0.0019, -0.0176]], dtype=np.float32)
MARKER_SIZE = 0.1 

# ==========================================
# --- CÁC CLASS HỖ TRỢ ---
# ==========================================

class AsyncLogger(threading.Thread):
    def __init__(self, filename, header):
        super().__init__(daemon=True)
        self.queue = queue.Queue()
        self.filename = filename
        self.header = header
        self.running = True
        
        # Tạo file và ghi header
        with open(self.filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(self.header)

    def log(self, data):
        if self.running:
            self.queue.put(data)

    def run(self):
        with open(self.filename, mode='a', newline='') as f:
            writer = csv.writer(f)
            while self.running or not self.queue.empty():
                try:
                    data = self.queue.get(timeout=0.5)
                    writer.writerow(data)
                    self.queue.task_done()
                except queue.Empty:
                    continue
                except Exception:
                    pass

    def stop(self):
        self.running = False
        self.join()

class StreamPusher:
    def __init__(self, cmd, w, h):
        self.p = subprocess.Popen(cmd, stdin=subprocess.PIPE, bufsize=0)
        self.q = queue.Queue(maxsize=2) 
        self.running = True
        self.w = w
        self.h = h
        self.t = threading.Thread(target=self.work, daemon=True)
        self.t.start()

    def work(self):
        while self.running:
            try: 
                frame = self.q.get(timeout=1)
                # Zero-copy write
                self.p.stdin.write(frame.data) 
                self.q.task_done()
            except queue.Empty:
                continue
            except Exception:
                continue

    def write(self, frame): 
        if not self.running:
            return
            
        if frame.shape[1] != self.w or frame.shape[0] != self.h:
            frame_out = cv2.resize(frame, (self.w, self.h))
        else:
            frame_out = frame
            
        try:
            self.q.put_nowait(frame_out)
        except queue.Full:
            pass 

    def stop(self): 
        self.running = False
        self.t.join()
        if self.p.stdin:
            self.p.stdin.close()
            self.p.wait()

class CameraStream:
    def __init__(self, src):
        # 1. Pipeline Ưu tiên: Hardware Decoding (v4l2h264dec)
        hw_pipeline = (
            f"rtspsrc location={src} latency=0 ! "
            "rtph264depay ! h264parse ! "
            "v4l2h264dec capture-io-mode=4 ! "
            "queue max-size-buffers=1 leaky=downstream ! "
            "videoconvert ! video/x-raw, format=(string)BGR ! "
            "appsink sync=false drop=true max-buffers=1"
        )
        
        # 2. Pipeline Dự phòng: Software Decoding (avdec_h264)
        sw_pipeline = (
            f"rtspsrc location={src} latency=0 ! "
            "rtph264depay ! h264parse ! "
            "avdec_h264 ! "
            "queue max-size-buffers=1 leaky=downstream ! "
            "videoconvert ! video/x-raw, format=(string)BGR ! "
            "appsink sync=false drop=true max-buffers=1"
        )

        print(f">>> [Camera] Trying Hardware Decoding...")
        self.cap = cv2.VideoCapture(hw_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            print("⚠️ Hardware Decoder failed! Switching to Software Decoder...")
            self.cap = cv2.VideoCapture(sw_pipeline, cv2.CAP_GSTREAMER)
            
            if not self.cap.isOpened():
                print("❌ GStreamer failed. Using default backend.")
                self.cap = cv2.VideoCapture(src)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

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
                    self.ret = ret
                    self.frame = frame
            else:
                time.sleep(0.005)

    def read(self):
        with self.lock:
            if self.frame is not None:
                return self.ret, self.frame.copy()
            else:
                return self.ret, None

    def stop(self):
        self.running = False
        self.t.join()
        self.cap.release()

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
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
            crc &= 0xFFFF
        return data + struct.pack('<H', crc)

    def rotate(self, yaw, pitch):
        if time.time() - self.last_sent < 0.05:
            return 
        self.last_sent = time.time()
        self.seq += 1
        
        yaw_val = max(min(int(yaw), 100), -100)
        pitch_val = max(min(int(pitch), 100), -100)
        
        payload = struct.pack('<bb', yaw_val, pitch_val) 
        msg = b'\x55\x66\x01' + struct.pack('<H', len(payload)) + struct.pack('<H', self.seq) + b'\x07' + payload
        
        try:
            full_msg = msg + self._append_crc16(msg)[-2:]
            self.sock.sendto(full_msg, (self.ip, self.port))
        except Exception:
            pass

    def center(self):
        self.seq += 1
        p = b'\x01'
        msg = b'\x55\x66\x01' + struct.pack('<H', len(p)) + struct.pack('<H', self.seq) + b'\x08' + p
        try:
            full_msg = msg + self._append_crc16(msg)[-2:]
            self.sock.sendto(full_msg, (self.ip, self.port))
        except Exception:
            pass

    def stop(self):
        self.rotate(0, 0)

# --- UTILS ---
def get_ffmpeg_command(w, h, fps):
    return [
        'ffmpeg', '-y', 
        '-f', 'rawvideo', 
        '-vcodec', 'rawvideo', 
        '-pix_fmt', 'bgr24', 
        '-s', f'{w}x{h}', 
        '-r', str(fps), 
        '-i', '-',
        '-c:v', 'h264_v4l2m2m', 
        '-b:v', '2000k', 
        '-pix_fmt', 'yuv420p', 
        '-g', '30', 
        '-preset', 'ultrafast', 
        '-tune', 'zerolatency',
        '-f', 'rtsp', RTSP_PUSH_URL
    ]

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, dist):
    marker_points = np.array([
        [-marker_size/2, marker_size/2, 0], 
        [marker_size/2, marker_size/2, 0],
        [marker_size/2, -marker_size/2, 0], 
        [-marker_size/2, -marker_size/2, 0]
    ], dtype=np.float32)
    
    rvecs = []
    tvecs = []
    
    for c in corners:
        _, r, t = cv2.solvePnP(marker_points, c, mtx, dist, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(r.reshape(1, 3))
        tvecs.append(t.reshape(1, 3))
        
    return rvecs, tvecs

def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    return np.array([
        math.atan2(R[2,1], R[2,2]), 
        math.atan2(-R[2,0], sy), 
        math.atan2(R[1,0], R[0,0])
    ])

# ==========================================
# --- MAIN PROGRAM ---
# ==========================================
running_global = True

def gps_thread():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", GPS_PORT))
    s.settimeout(1.0)
    
    while running_global:
        try:
            s.recvfrom(1024)
        except Exception:
            pass

def main():
    global running_global
    print(f">>> PI TRACKING V6 (CLEAN FORMAT & HW ACCELERATED)...")
    
    # 1. Init Camera
    cam = CameraStream(CAMERA_URL)
    time.sleep(2.0) 
    
    if not cam.ret:
        print("❌ Camera not ready!")
        cam.stop()
        return

    # 2. Init Subsystems
    pusher = StreamPusher(get_ffmpeg_command(STREAM_W, STREAM_H, FIXED_FPS), STREAM_W, STREAM_H)
    gimbal = SiyiGimbal(SIYI_IP, SIYI_PORT)
    gimbal.center()
    
    # 3. Init Aruco
    try:
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        params = aruco.DetectorParameters_create()
    except AttributeError:
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        params = aruco.DetectorParameters()
    
    params.adaptiveThreshWinSizeStep = 10
    
    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(aruco_dict, params)
    else:
        detector = None

    # 4. Init Logger
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_cols = ["Timestamp", "Algo_ms", "Loop_ms", "FPS", "Pos_X", "Pos_Y", "Pos_Z", "Roll", "Pitch", "Yaw"]
    logger = AsyncLogger(f"track_v6_hw_{timestamp_str}.csv", log_cols)
    logger.start()

    # 5. Loop Variables
    fc = 0
    st = time.perf_counter()
    fps = 0
    center = (INPUT_W // 2, INPUT_H // 2) 
    lc = None
    lid = None
    ly = 0
    lp = 0
    dc = 0 
    
    print(">>> SYSTEM LIVE!")

    while True:
        try:
            ls = time.perf_counter()
            
            # --- Read Frame ---
            ret, frame = cam.read()
            if not ret or frame is None:
                time.sleep(0.001)
                continue

            tsa = time.perf_counter()
            dc += 1
            
            # --- Detect ArUco (Skipping frames logic) ---
            if (dc % (SKIP_FRAME + 1) == 0):
                sw = int(INPUT_W * DETECT_SCALE)
                sh = int(INPUT_H * DETECT_SCALE)
                small = cv2.resize(frame, (sw, sh), interpolation=cv2.INTER_NEAREST)
                gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
                
                if detector:
                    corners, ids, _ = detector.detectMarkers(gray)
                else:
                    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)
                
                if ids is not None and len(ids) > 0:
                    corners = tuple((c / DETECT_SCALE).astype(np.float32) for c in corners)
                    lc = corners
                    lid = ids
                else:
                    lid = None
            else:
                corners = lc
                ids = lid

            # --- Pose Estimation & Control ---
            px, py, pz, r, p, y = 0, 0, 0, 0, 0, 0
            is_tracking = False

            if lid is not None and lc is not None:
                is_tracking = True
                try:
                    rvecs, tvecs = my_estimatePoseSingleMarkers(lc, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS)
                    px, py, pz = tvecs[0][0]
                    
                    rmat, _ = cv2.Rodrigues(rvecs[0])
                    euler = rotationMatrixToEulerAngles(rmat)
                    r, p, y = [math.degrees(x) for x in euler]
                    
                    cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, rvecs[0], tvecs[0], 0.05)
                    aruco.drawDetectedMarkers(frame, lc, lid)
                except Exception:
                    pass

                # PID Calculation
                c = lc[0][0]
                cx = int((c[0][0] + c[2][0]) / 2)
                cy = int((c[0][1] + c[2][1]) / 2)
                
                ex = cx - center[0]
                ey = cy - center[1]
                
                if abs(ex) > DEADZONE:
                    ly = int(ex * KP_YAW)
                else:
                    ly = 0
                    
                if abs(ey) > DEADZONE:
                    lp = int(-ey * KP_PITCH)
                else:
                    lp = 0
                    
                cv2.line(frame, center, (cx, cy), (0, 0, 255), 2)
            else:
                ly = 0
                lp = 0

            # --- Gimbal Action ---
            if TRACKING_ACTIVE:
                gimbal.rotate(ly, lp)
            
            algo_ms = (time.perf_counter() - tsa) * 1000

            # --- Draw OSD ---
            cv2.drawMarker(frame, center, (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
            cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            if is_tracking:
                cv2.putText(frame, f"X:{px:.2f} Y:{py:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # --- Push Stream ---
            pusher.write(frame)

            # --- FPS Calculation & Logging ---
            el = time.perf_counter() - ls
            ld = el * 1000
            wait = FRAME_TIME_MS - el
            
            if wait > 0:
                time.sleep(wait)
            
            fc += 1
            if fc >= 10:
                now = time.perf_counter()
                fps = fc / (now - st)
                st = now
                fc = 0
                
                log_data = [
                    datetime.now().strftime("%H:%M:%S.%f")[:-3], 
                    f"{algo_ms:.2f}", 
                    f"{ld:.2f}", 
                    f"{fps:.2f}", 
                    f"{px:.3f}", f"{py:.3f}", f"{pz:.3f}", 
                    f"{r:.2f}", f"{p:.2f}", f"{y:.2f}"
                ]
                logger.log(log_data)
        
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")
            break

    # --- Cleanup ---
    running_global = False
    cam.stop()
    pusher.stop()
    gimbal.stop()
    logger.stop()
    print(">>> SHUTDOWN COMPLETE")

if __name__ == "__main__":
    t = threading.Thread(target=gps_thread, daemon=True)
    t.start()
    main()
