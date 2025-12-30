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
DETECT_SCALE = 0.7  
SKIP_FRAME = 1      

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
KP_YAW, KP_PITCH, DEADZONE = 0.15, 0.25, 15     

# --- CAMERA MATRIX ---
CAMERA_MATRIX = np.array([[717.14, 0, 664.29], [0, 717.88, 354.24], [0, 0, 1]], dtype=np.float32)
DIST_COEFFS = np.array([[-0.0764, 0.0742, -0.0013, 0.0019, -0.0176]], dtype=np.float32)
MARKER_SIZE = 0.1 

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

def isRotationMatrix(R): return np.linalg.norm(np.identity(3, dtype=R.dtype) - np.dot(R.T, R)) < 1e-6
def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0,0]*R[0,0] + R[1,0]*R[1,0])
    return np.array([math.atan2(R[2,1], R[2,2]), math.atan2(-R[2,0], sy), math.atan2(R[1,0], R[0,0])])

# --- FFMPEG OUTPUT ---
def get_ffmpeg_command(w, h, fps):
    return ['ffmpeg', '-y', '-f', 'rawvideo', '-vcodec', 'rawvideo', '-pix_fmt', 'bgr24', 
            '-s', f'{w}x{h}', '-r', str(fps), '-i', '-',
            '-c:v', 'h264_v4l2m2m', '-b:v', '2000k', '-pix_fmt', 'yuv420p', '-g', '30', '-bufsize', '1000k',   
            '-f', 'rtsp', RTSP_PUSH_URL]

# --- CLASSES ---
class SiyiGimbal:
    def __init__(self, ip, port):
        self.ip = ip; self.port = port; self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); self.seq = 0; self.last_sent = 0
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
    def center(self): self.seq+=1; p=b'\x01'; m=b'\x55\x66\x01'+struct.pack('<H',len(p))+struct.pack('<H',self.seq)+b'\x08'+p; self.sock.sendto(m+self._append_crc16(m)[-2:],(self.ip,self.port))
    def stop(self): self.rotate(0,0)

# --- CAMERA STREAM (GSTREAMER FIX) ---
class CameraStream:
    def __init__(self, src):
        # PIPELINE CHUẨN ĐÃ ĐƯỢC TEST TRÊN CLI:
        # Thêm 'queue' để tránh drop frame khi decode
        # Sửa format=(string)BGR cho đúng chuẩn OpenCV Python
        gst_pipeline = (
            f"rtspsrc location={src} latency=0 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! "
            "queue ! "  # <--- QUAN TRỌNG: Đệm dữ liệu sau khi giải mã
            "videoconvert ! video/x-raw, format=(string)BGR ! "
            "appsink sync=false drop=true max-buffers=1"
        )

        print(f">>> [Camera] Attempting GStreamer Pipeline:\n{gst_pipeline}")
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        if self.cap.isOpened():
            self.mode = "GSTREAMER"
            print("✅ GStreamer Initialized Successfully!")
        else:
            print("❌ GStreamer Failed via Python (OpenCV might miss GStreamer support).")
            print("⚠️ Switching to Threaded Bufferless Mode (Standard API)...")
            self.cap = cv2.VideoCapture(src)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.mode = "THREADED_BUFFERLESS"

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
running_global = True; current_lat, current_lon = 0.0, 0.0
def gps_thread():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("0.0.0.0", GPS_PORT)); s.settimeout(1.0)
    while running_global:
        try: d, _ = s.recvfrom(1024); _, _, current_lat, current_lon, _ = struct.unpack('qi3d', d)
        except: pass

def main():
    global running_global
    print(f">>> PI TRACKING V4 (Final Attempt)...")
    cam = CameraStream(CAMERA_URL)
    time.sleep(2.0)
    if not cam.ret: print("❌ Camera not ready!"); cam.stop(); return
    
    pusher = StreamPusher(get_ffmpeg_command(STREAM_W, STREAM_H, FIXED_FPS), STREAM_W, STREAM_H)
    gimbal = SiyiGimbal(SIYI_IP, SIYI_PORT); gimbal.center()
    
    try: aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50); params = aruco.DetectorParameters_create()
    except: aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50); params = aruco.DetectorParameters()
    params.adaptiveThreshWinSizeStep = 10
    detector = aruco.ArucoDetector(aruco_dict, params) if hasattr(aruco, "ArucoDetector") else None

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_file = open(f"track_v4_log_{timestamp_str}.csv", mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Timestamp", "Algo_ms", "Loop_ms", "FPS", "Pos_X", "Pos_Y", "Pos_Z", "Roll", "Pitch", "Yaw"])

    fc = 0; st = time.perf_counter(); fps = 0; ld = 0 
    center = (STREAM_W//2, STREAM_H//2); lc = None; lid = None; ly = 0; lp = 0; dc = 0 
    
    print(">>> SYSTEM LIVE!")

    while True:
        ls = time.perf_counter()
        ret, frame = cam.read()
        if not ret or frame is None: time.sleep(0.001); continue

        tsa = time.perf_counter()
        dc += 1
        should_detect = (dc % (SKIP_FRAME + 1) == 0)

        if should_detect:
            sw, sh = int(STREAM_W * DETECT_SCALE), int(STREAM_H * DETECT_SCALE)
            small = cv2.resize(frame, (sw, sh), interpolation=cv2.INTER_NEAREST)
            gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
            if detector: corners, ids, _ = detector.detectMarkers(gray)
            else: corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)
            
            if ids is not None and len(ids) > 0:
                corners = tuple((c / DETECT_SCALE).astype(np.float32) for c in corners)
                lc, lid = corners, ids
            else: lid = None
        else: corners, ids = lc, lid

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
                cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, rvecs[0], tvecs[0], 0.05)
                aruco.drawDetectedMarkers(frame, lc, lid)
            except: pass

            c = lc[0][0]; cx, cy = int((c[0][0]+c[2][0])/2), int((c[0][1]+c[2][1])/2)
            ex, ey = cx - center[0], cy - center[1]
            ly = int(ex*KP_YAW) if abs(ex)>DEADZONE else 0
            lp = int(-ey*KP_PITCH) if abs(ey)>DEADZONE else 0
            cv2.line(frame, center, (cx, cy), (0,0,255), 2)
        else: ly, lp = 0, 0

        if TRACKING_ACTIVE: gimbal.rotate(ly, lp)
        algo_ms = (time.perf_counter() - tsa) * 1000

        cv2.drawMarker(frame, center, (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
        cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if is_tracking: cv2.putText(frame, f"X:{px:.2f} Y:{py:.2f} Z:{pz:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        pusher.write(frame)

        el = time.perf_counter() - ls; ld = el*1000; wait = FRAME_TIME_MS - el
        if wait > 0: time.sleep(wait)
        
        fc += 1
        if fc >= 10:
            now = time.perf_counter(); fps = fc / (now - st); st = now; fc = 0
            try: csv_writer.writerow([datetime.now().strftime("%H:%M:%S.%f")[:-3], f"{algo_ms:.2f}", f"{ld:.2f}", f"{fps:.2f}", f"{px:.3f}", f"{py:.3f}", f"{pz:.3f}", f"{r:.2f}", f"{p:.2f}", f"{y:.2f}"])
            except: pass

    running_global = False; cam.stop(); pusher.stop(); gimbal.stop(); csv_file.close()

if __name__ == "__main__": t = threading.Thread(target=gps_thread, daemon=True); t.start(); main()
