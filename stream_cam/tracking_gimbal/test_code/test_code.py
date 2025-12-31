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
import numpy as np
import math

# ==========================================
# --- CẤU HÌNH HỆ THỐNG ---
# ==========================================
DETECT_SCALE = 0.6      
SKIP_FRAME = 2          
MAX_LOST_FRAMES = 5     # FIX: Giảm từ 15 xuống 5 để dừng nhanh hơn khi mất dấu

RTSP_PUSH_URL = "rtsp://localhost:8554/siyi_aruco"
CAMERA_URL = "rtsp://127.0.0.1:8554/my_camera" 

GPS_PORT = 5555
STREAM_W, STREAM_H = 1280, 720 
FIXED_FPS = 30   
FRAME_TIME_MS = 1.0 / FIXED_FPS 

# --- CẤU HÌNH TRACKING ---
SIYI_IP = "192.168.168.14"
SIYI_PORT = 37260
TRACKING_ACTIVE = True
KP_YAW = 0.12     
KP_PITCH = 0.20
DEADZONE = 10     

# --- CAMERA MATRIX ---
CAMERA_MATRIX = np.array([[717.14, 0, 664.29], [0, 717.88, 354.24], [0, 0, 1]], dtype=np.float32)
DIST_COEFFS = np.array([[-0.0764, 0.0742, -0.0013, 0.0019, -0.0176]], dtype=np.float32)
MARKER_SIZE = 0.1 

# ==========================================
# --- LỚP KALMAN FILTER (CÓ PHANH) ---
# ==========================================
class KalmanTracker:
    def __init__(self, center_x, center_y):
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
        self.kf.transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], np.float32)
        
        # Tăng Q nhẹ để phản ứng nhanh, R giữ nguyên
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.05
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.1 
        
        self.kf.statePost = np.array([center_x, center_y, 0, 0], dtype=np.float32)
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)

    def update(self, x, y):
        # Khi CÓ marker: Cập nhật vị trí thực
        measurement = np.array([[np.float32(x)], [np.float32(y)]])
        self.kf.correct(measurement)
        prediction = self.kf.predict()
        return prediction[0][0], prediction[1][0]

    def predict_only(self):
        # FIX: VELOCITY DECAY (PHANH DẦN)
        # Lấy trạng thái hiện tại
        state = self.kf.statePost
        # Nhân vận tốc (dx, dy) với 0.5 để giảm tốc độ dự đoán
        state[2] *= 0.5 
        state[3] *= 0.5
        self.kf.statePost = state
        
        prediction = self.kf.predict()
        return prediction[0][0], prediction[1][0]
    
    def reset_velocity(self):
        # Reset vận tốc về 0 (dùng khi mất tracking quá lâu)
        self.kf.statePost[2] = 0
        self.kf.statePost[3] = 0

# --- ESTIMATE POSE ---
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

# --- FFMPEG OUTPUT ---
def get_ffmpeg_command(w, h, fps):
    return ['ffmpeg', '-y', '-f', 'rawvideo', '-vcodec', 'rawvideo', '-pix_fmt', 'bgr24', 
            '-s', f'{w}x{h}', '-r', str(fps), '-i', '-',
            '-c:v', 'h264_v4l2m2m', '-b:v', '2000k', '-pix_fmt', 'yuv420p', '-g', '30', '-bufsize', '1000k',   
            '-f', 'rtsp', RTSP_PUSH_URL]

# --- SIYI GIMBAL ---
class SiyiGimbal:
    def __init__(self, ip, port):
        self.ip = ip; self.port = port; self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0; self.last_sent = 0
    def _append_crc16(self, data):
        crc = 0
        for byte in data:
            crc ^= byte << 8
            for _ in range(8): crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1; crc &= 0xFFFF
        return data + struct.pack('<H', crc)
    def rotate(self, yaw, pitch):
        # Giảm rate limit xuống một chút để lệnh STOP được gửi nhanh hơn
        if time.time() - self.last_sent < 0.02: return 
        self.last_sent = time.time(); self.seq += 1
        payload = struct.pack('<bb', max(min(int(yaw),100),-100), max(min(int(pitch),100),-100)) 
        msg = b'\x55\x66\x01' + struct.pack('<H', len(payload)) + struct.pack('<H', self.seq) + b'\x07' + payload
        try: self.sock.sendto(msg + self._append_crc16(msg)[-2:], (self.ip, self.port))
        except: pass
    def center(self): self.seq+=1; p=b'\x01'; m=b'\x55\x66\x01'+struct.pack('<H',len(p))+struct.pack('<H',self.seq)+b'\x08'+p; self.sock.sendto(m+self._append_crc16(m)[-2:],(self.ip,self.port))
    def stop(self): self.rotate(0,0)

# --- CAMERA STREAM ---
class CameraStream:
    def __init__(self, src):
        gst_pipeline = (
            f"rtspsrc location={src} latency=0 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! "
            "queue max-size-buffers=1 leaky=downstream ! " 
            "videoconvert ! video/x-raw, format=(string)BGR ! "
            "appsink sync=false drop=true max-buffers=1"
        )
        print(f">>> [Camera] Pipeline: {gst_pipeline}")
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            print("❌ GStreamer Failed. Fallback to standard.")
            self.cap = cv2.VideoCapture(src)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.ret, self.frame = False, None
        self.running = True
        self.lock = threading.Lock()
        self.t = threading.Thread(target=self.update, args=(), daemon=True); self.t.start()

    def update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock: self.ret, self.frame = ret, frame
            else: time.sleep(0.005)
    def read(self):
        with self.lock: return self.ret, self.frame.copy() if self.frame is not None else None
    def stop(self): self.running = False; self.t.join(); self.cap.release()

class StreamPusher:
    def __init__(self, cmd, w, h):
        self.p = subprocess.Popen(cmd, stdin=subprocess.PIPE); self.q = queue.Queue(maxsize=2); self.r = True; self.w, self.h = w, h
        self.t = threading.Thread(target=self.work, daemon=True); self.t.start()
    def work(self):
        while self.r:
            try: 
                f = self.q.get(timeout=1); h, w = f.shape[:2]
                if w!=self.w or h!=self.h: f = cv2.resize(f, (self.w, self.h))
                self.p.stdin.write(f.tobytes()); self.q.task_done()
            except: continue
    def write(self, f): 
        if self.r: 
            try: self.q.put(f, block=False)
            except: pass
    def stop(self): self.r = False; self.t.join(); self.p.stdin.close(); self.p.wait()

# --- MAIN ---
running_global = True
def main():
    global running_global
    print(f">>> PI TRACKING V6 (Safety & Brake)...")
    cam = CameraStream(CAMERA_URL)
    time.sleep(2.0)
    if not cam.ret: print("❌ Camera not ready!"); cam.stop(); return
    
    pusher = StreamPusher(get_ffmpeg_command(STREAM_W, STREAM_H, FIXED_FPS), STREAM_W, STREAM_H)
    gimbal = SiyiGimbal(SIYI_IP, SIYI_PORT); gimbal.center()
    
    try: aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50); params = aruco.DetectorParameters_create()
    except: aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50); params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, params) if hasattr(aruco, "ArucoDetector") else None

    # Logging
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_file = open(f"track_v6_log_{timestamp_str}.csv", mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Timestamp", "FPS", "Tracking", "Target_X", "Target_Y", "Pos_Z"])

    fc = 0; st = time.perf_counter(); fps = 0
    center_img = (STREAM_W//2, STREAM_H//2)
    
    lost_frames = 0
    is_tracking_active = False
    kalman = KalmanTracker(center_img[0], center_img[1])
    
    last_corners = None
    target_x, target_y = center_img[0], center_img[1]
    
    print(">>> SYSTEM LIVE!")

    while True:
        ls = time.perf_counter()
        ret, frame = cam.read()
        if not ret or frame is None: time.sleep(0.001); continue

        # --- 1. DETECTION LOGIC ---
        should_detect = (fc % (SKIP_FRAME + 1) == 0)
        detected_now = False

        if should_detect:
            sw, sh = int(STREAM_W * DETECT_SCALE), int(STREAM_H * DETECT_SCALE)
            small = cv2.resize(frame, (sw, sh), interpolation=cv2.INTER_NEAREST)
            gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
            
            if detector: corners, ids, _ = detector.detectMarkers(gray)
            else: corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)
            
            if ids is not None and len(ids) > 0:
                corners = tuple((c / DETECT_SCALE).astype(np.float32) for c in corners)
                last_corners, ids = corners, ids
                c = corners[0][0]
                raw_cx, raw_cy = (c[0][0]+c[2][0])/2, (c[0][1]+c[2][1])/2
                
                target_x, target_y = kalman.update(raw_cx, raw_cy)
                detected_now = True
                lost_frames = 0
                is_tracking_active = True
            else:
                detected_now = False
                lost_frames += 1
        else:
            # --- FRAME SKIP: PREDICT ONLY ---
            target_x, target_y = kalman.predict_only()
            
            # FIX: BOUNDARY CHECK - Nếu bay ra khỏi khung hình -> Coi như mất luôn
            if not (0 <= target_x <= STREAM_W and 0 <= target_y <= STREAM_H):
                lost_frames = MAX_LOST_FRAMES + 1
            else:
                if lost_frames < MAX_LOST_FRAMES:
                    lost_frames += 1
                else:
                    is_tracking_active = False

        # --- SAFETY CHECK ---
        if lost_frames > MAX_LOST_FRAMES:
            is_tracking_active = False
            last_corners = None
            kalman.reset_velocity() # Xóa vận tốc cũ để tránh "nhớ" đà quay

        # --- 2. GIMBAL CONTROL ---
        ly, lp, pz = 0, 0, 0
        
        if is_tracking_active:
            ex = target_x - center_img[0]
            ey = target_y - center_img[1]
            
            ly = int(ex * KP_YAW) if abs(ex) > DEADZONE else 0
            lp = int(-ey * KP_PITCH) if abs(ey) > DEADZONE else 0
            
            if TRACKING_ACTIVE:
                gimbal.rotate(ly, lp)
            
            cv2.circle(frame, (int(target_x), int(target_y)), 10, (0, 255, 255), 2)
            cv2.line(frame, center_img, (int(target_x), int(target_y)), (0, 255, 255), 1)
            
            if detected_now and last_corners is not None:
                try:
                    rvecs, tvecs = my_estimatePoseSingleMarkers(last_corners, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS)
                    pz = tvecs[0][0][2]
                    cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, rvecs[0], tvecs[0], 0.05)
                except: pass
        else:
            # FIX: ACTIVE BRAKE - Gửi lệnh dừng liên tục khi mất dấu
            if TRACKING_ACTIVE:
                gimbal.rotate(0, 0)
            
            cv2.putText(frame, "STOPPED", (center_img[0]-50, center_img[1]-20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # --- 3. DISPLAY & LOG ---
        cv2.drawMarker(frame, center_img, (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(frame, f"FPS: {int(fps)} | L: {lost_frames}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        pusher.write(frame)

        el = time.perf_counter() - ls
        wait = FRAME_TIME_MS - el
        if wait > 0: time.sleep(wait)
        
        fc += 1
        if fc >= 10:
            now = time.perf_counter(); fps = fc / (now - st); st = now; fc = 0
            try: csv_writer.writerow([datetime.now().strftime("%H:%M:%S.%f")[:-3], f"{fps:.2f}", is_tracking_active, f"{target_x:.1f}", f"{target_y:.1f}", f"{pz:.2f}"])
            except: pass

    running_global = False; cam.stop(); pusher.stop(); gimbal.stop(); csv_file.close()

if __name__ == "__main__": main()
