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
DETECT_SCALE = 0.7       # Scale ảnh để detect ArUco (giữ nguyên để tối ưu detect)
SKIP_FRAME = 1           # Bỏ qua bao nhiêu frame không detect (1 = 30fps detect 15fps)

# URL Streaming
RTSP_PUSH_URL = "rtsp://localhost:8554/siyi_aruco"
CAMERA_URL = "rtsp://127.0.0.1:8554/my_camera" 

# Cấu hình kích thước ảnh xử lý và stream
INPUT_W, INPUT_H = 1280, 720   # Kích thước ảnh gốc từ Camera
STREAM_SCALE_OUT = 0.5         # Tỷ lệ ảnh stream ra (1.0 = giữ nguyên, 0.5 = giảm 1/2 để nhẹ hơn)
STREAM_W = int(INPUT_W * STREAM_SCALE_OUT)
STREAM_H = int(INPUT_H * STREAM_SCALE_OUT)

FIXED_FPS = 30   
FRAME_TIME_MS = 1.0 / FIXED_FPS 
GPS_PORT = 5555

# --- CẤU HÌNH TRACKING ---
SIYI_IP = "192.168.168.14"
SIYI_PORT = 37260
TRACKING_ACTIVE = True
KP_YAW, KP_PITCH, DEADZONE = 0.15, 0.25, 15     

# --- CAMERA MATRIX (Calibrated) ---
CAMERA_MATRIX = np.array([[717.14, 0, 664.29], [0, 717.88, 354.24], [0, 0, 1]], dtype=np.float32)
DIST_COEFFS = np.array([[-0.0764, 0.0742, -0.0013, 0.0019, -0.0176]], dtype=np.float32)
MARKER_SIZE = 0.1 

# ==========================================
# --- CÁC CLASS HỖ TRỢ (TỐI ƯU HÓA) ---
# ==========================================

# 1. ASYNC LOGGER (Tách luồng I/O)
class AsyncLogger(threading.Thread):
    def __init__(self, filename, header):
        super().__init__(daemon=True)
        self.queue = queue.Queue()
        self.filename = filename
        self.header = header
        self.running = True
        
        # Tạo file và ghi header ngay lập tức
        with open(self.filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(self.header)

    def log(self, data):
        if self.running:
            self.queue.put(data)

    def run(self):
        # Mở file ở chế độ append (a) để ghi liên tục
        with open(self.filename, mode='a', newline='') as f:
            writer = csv.writer(f)
            while self.running or not self.queue.empty():
                try:
                    data = self.queue.get(timeout=0.5)
                    writer.writerow(data)
                    self.queue.task_done()
                except queue.Empty:
                    continue
                except Exception as e:
                    print(f"Log Error: {e}")

    def stop(self):
        self.running = False
        self.join()

# 2. STREAM PUSHER (Tối ưu bộ nhớ - Zero Copy)
class StreamPusher:
    def __init__(self, cmd, w, h):
        self.p = subprocess.Popen(cmd, stdin=subprocess.PIPE, bufsize=0)
        self.q = queue.Queue(maxsize=2) # Giữ buffer thấp để giảm latency
        self.running = True
        self.w, self.h = w, h
        self.t = threading.Thread(target=self.work, daemon=True)
        self.t.start()

    def work(self):
        while self.running:
            try: 
                frame = self.q.get(timeout=1)
                # Tối ưu: Dùng frame.data thay vì frame.tobytes() để tránh copy bộ nhớ
                # Lưu ý: Frame phải là C-contiguous (mặc định OpenCV là có)
                self.p.stdin.write(frame.data) 
                self.q.task_done()
            except queue.Empty:
                continue
            except BrokenPipeError:
                print("❌ FFMPEG Pipe broken!")
                self.running = False
            except Exception as e:
                print(f"Stream Error: {e}")

    def write(self, frame): 
        if not self.running: return
        
        # Chỉ resize nếu kích thước khác cấu hình đầu ra
        if frame.shape[1] != self.w or frame.shape[0] != self.h:
            frame_out = cv2.resize(frame, (self.w, self.h))
        else:
            frame_out = frame

        try: 
            # Dùng put_nowait để drop frame nếu ffmpeg xử lý không kịp (tránh lag dây chuyền)
            self.q.put_nowait(frame_out)
        except queue.Full: 
            pass 

    def stop(self): 
        self.running = False
        self.t.join()
        if self.p.stdin: self.p.stdin.close()
        self.p.wait()

# 3. CAMERA STREAM (Tối ưu Locking)
class CameraStream:
    def __init__(self, src):
        gst_pipeline = (
            f"rtspsrc location={src} latency=0 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! "
            "queue max-size-buffers=1 leaky=downstream ! " # Queue nhỏ, drop frame cũ
            "videoconvert ! video/x-raw, format=(string)BGR ! "
            "appsink sync=false drop=true max-buffers=1"
        )
        print(f">>> [Camera] Pipeline: {gst_pipeline}")
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            print("⚠️ Fallback to Standard Capture")
            self.cap = cv2.VideoCapture(src)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.ret, self.frame = False, None
        self.running = True
        self.lock = threading.Lock()
        self.t = threading.Thread(target=self.update, args=(), daemon=True)
        self.t.start()

    def update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                # Tối ưu: Chỉ giữ lock khi gán tham chiếu, rất nhanh
                with self.lock: 
                    self.ret = ret
                    self.frame = frame # Gán pointer, không copy dữ liệu
            else: 
                time.sleep(0.005)

    def read(self):
        # Tối ưu: Lấy tham chiếu trong lock, copy ngoài lock
        with self.lock: 
            ret = self.ret
            frame_ref = self.frame 
        
        if frame_ref is not None:
            return ret, frame_ref.copy() # Copy an toàn ở đây
        return ret, None

    def stop(self): 
        self.running = False
        self.t.join()
        self.cap.release()

# 4. GIMBAL CONTROL (Giữ nguyên)
class SiyiGimbal:
    def __init__(self, ip, port):
        self.ip = ip; self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0; self.last_sent = 0

    def _append_crc16(self, data):
        crc = 0
        for byte in data:
            crc ^= byte << 8
            for _ in range(8): crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1; crc &= 0xFFFF
        return data + struct.pack('<H', crc)

    def rotate(self, yaw, pitch):
        if time.time() - self.last_sent < 0.05: return 
        self.last_sent = time.time(); self.seq += 1
        payload = struct.pack('<bb', max(min(int(yaw),100),-100), max(min(int(pitch),100),-100)) 
        msg = b'\x55\x66\x01' + struct.pack('<H', len(payload)) + struct.pack('<H', self.seq) + b'\x07' + payload
        try: self.sock.sendto(msg + self._append_crc16(msg)[-2:], (self.ip, self.port))
        except: pass

    def center(self): 
        self.seq+=1; p=b'\x01'
        m=b'\x55\x66\x01'+struct.pack('<H',len(p))+struct.pack('<H',self.seq)+b'\x08'+p
        try: self.sock.sendto(m+self._append_crc16(m)[-2:],(self.ip,self.port))
        except: pass
    
    def stop(self): self.rotate(0,0)

# --- UTILS ---
def get_ffmpeg_command(w, h, fps):
    # Sử dụng preset ultrafast để giảm CPU khi encode lại
    return ['ffmpeg', '-y', '-f', 'rawvideo', '-vcodec', 'rawvideo', '-pix_fmt', 'bgr24', 
            '-s', f'{w}x{h}', '-r', str(fps), '-i', '-',
            '-c:v', 'h264_v4l2m2m', '-b:v', '2000k', '-pix_fmt', 'yuv420p', 
            '-g', '30', '-preset', 'ultrafast', '-tune', 'zerolatency',
            '-f', 'rtsp', RTSP_PUSH_URL]

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, dist):
    marker_points = np.array([
        [-marker_size/2, marker_size/2, 0], [marker_size/2, marker_size/2, 0],
        [marker_size/2, -marker_size/2, 0], [-marker_size/2, -marker_size/2, 0]
    ], dtype=np.float32)
    rvecs, tvecs = [], []
    for c in corners:
        _, r, t = cv2.solvePnP(marker_points, c, mtx, dist, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(r.reshape(1, 3)); tvecs.append(t.reshape(1, 3))
    return rvecs, tvecs

def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0,0]*R[0,0] + R[1,0]*R[1,0])
    return np.array([math.atan2(R[2,1], R[2,2]), math.atan2(-R[2,0], sy), math.atan2(R[1,0], R[0,0])])

# ==========================================
# --- MAIN PROGRAM ---
# ==========================================
running_global = True
current_lat, current_lon = 0.0, 0.0

def gps_thread():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("0.0.0.0", GPS_PORT)); s.settimeout(1.0)
    while running_global:
        try: d, _ = s.recvfrom(1024); _, _, current_lat, current_lon, _ = struct.unpack('qi3d', d)
        except: pass

def main():
    global running_global
    print(f">>> PI TRACKING V5 (OPTIMIZED)...")
    
    # Init Camera
    cam = CameraStream(CAMERA_URL)
    time.sleep(2.0) # Warmup
    if not cam.ret: print("❌ Camera not ready!"); cam.stop(); return

    # Init Subsystems
    pusher = StreamPusher(get_ffmpeg_command(STREAM_W, STREAM_H, FIXED_FPS), STREAM_W, STREAM_H)
    gimbal = SiyiGimbal(SIYI_IP, SIYI_PORT); gimbal.center()
    
    # Init Aruco
    try: aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50); params = aruco.DetectorParameters_create()
    except: aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50); params = aruco.DetectorParameters()
    params.adaptiveThreshWinSizeStep = 10
    detector = aruco.ArucoDetector(aruco_dict, params) if hasattr(aruco, "ArucoDetector") else None

    # Init Async Logger
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_header = ["Timestamp", "Algo_ms", "Loop_ms", "FPS", "Pos_X", "Pos_Y", "Pos_Z", "Roll", "Pitch", "Yaw"]
    logger = AsyncLogger(f"track_v5_log_{timestamp_str}.csv", log_header)
    logger.start()

    # Variables
    fc = 0; st = time.perf_counter(); fps = 0; ld = 0 
    center = (INPUT_W//2, INPUT_H//2) 
    lc = None; lid = None; ly = 0; lp = 0; dc = 0 
    
    print(">>> SYSTEM LIVE & OPTIMIZED!")

    while True:
        try:
            ls = time.perf_counter()
            
            # 1. Đọc Camera (Thread-safe, optimized)
            ret, frame = cam.read()
            if not ret or frame is None: 
                time.sleep(0.001); continue

            tsa = time.perf_counter()
            dc += 1
            should_detect = (dc % (SKIP_FRAME + 1) == 0)

            # 2. Detect ArUco
            if should_detect:
                # Resize cho detect (nhanh hơn dùng ảnh gốc)
                sw, sh = int(INPUT_W * DETECT_SCALE), int(INPUT_H * DETECT_SCALE)
                small = cv2.resize(frame, (sw, sh), interpolation=cv2.INTER_NEAREST)
                gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
                
                if detector: corners, ids, _ = detector.detectMarkers(gray)
                else: corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)
                
                if ids is not None and len(ids) > 0:
                    # Scale lại tọa độ về ảnh gốc
                    corners = tuple((c / DETECT_SCALE).astype(np.float32) for c in corners)
                    lc, lid = corners, ids
                else: lid = None
            else: 
                corners, ids = lc, lid # Dùng lại kết quả frame trước

            # 3. Tính toán Pose & PID
            px, py, pz, r, p, y = 0,0,0,0,0,0
            is_tracking = False

            if lid is not None and lc is not None:
                is_tracking = True
                try:
                    rvecs, tvecs = my_estimatePoseSingleMarkers(lc, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS)
                    px, py, pz = tvecs[0][0]; 
                    rmat, _ = cv2.Rodrigues(rvecs[0])
                    euler = rotationMatrixToEulerAngles(rmat)
                    r, p, y = [math.degrees(x) for x in euler]
                    
                    # Vẽ lên frame gốc
                    cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, rvecs[0], tvecs[0], 0.05)
                    aruco.drawDetectedMarkers(frame, lc, lid)
                except: pass

                # PID Logic
                c = lc[0][0]; cx, cy = int((c[0][0]+c[2][0])/2), int((c[0][1]+c[2][1])/2)
                ex, ey = cx - center[0], cy - center[1]
                ly = int(ex*KP_YAW) if abs(ex)>DEADZONE else 0
                lp = int(-ey*KP_PITCH) if abs(ey)>DEADZONE else 0
                cv2.line(frame, center, (cx, cy), (0,0,255), 2)
            else: ly, lp = 0, 0

            # 4. Điều khiển Gimbal
            if TRACKING_ACTIVE: gimbal.rotate(ly, lp)
            
            algo_ms = (time.perf_counter() - tsa) * 1000

            # 5. Vẽ OSD
            cv2.drawMarker(frame, center, (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
            cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if is_tracking: 
                cv2.putText(frame, f"X:{px:.2f} Y:{py:.2f} Z:{pz:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # 6. Stream Output (Async Push)
            pusher.write(frame)

            # 7. FPS Control & Logging
            el = time.perf_counter() - ls
            ld = el * 1000
            wait = FRAME_TIME_MS - el
            if wait > 0: time.sleep(wait)
            
            fc += 1
            if fc >= 10:
                now = time.perf_counter()
                fps = fc / (now - st)
                st = now; fc = 0
                # Đẩy data vào Queue log thay vì ghi trực tiếp
                log_data = [datetime.now().strftime("%H:%M:%S.%f")[:-3], f"{algo_ms:.2f}", f"{ld:.2f}", f"{fps:.2f}", f"{px:.3f}", f"{py:.3f}", f"{pz:.3f}", f"{r:.2f}", f"{p:.2f}", f"{y:.2f}"]
                logger.log(log_data)
        
        except KeyboardInterrupt:
            print("Stopping...")
            break
        except Exception as e:
            print(f"Main Loop Error: {e}")
            break

    # Cleanup
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
