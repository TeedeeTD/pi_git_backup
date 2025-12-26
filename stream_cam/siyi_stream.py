import sys
import time
import threading
import socket
import struct
import cv2
import cv2.aruco as aruco
import subprocess

# --- CẤU HÌNH ---

# 1. Output: Tên luồng video đã xử lý (đẩy vào MediaMTX để PC xem)
RTSP_PUSH_URL = "rtsp://localhost:8554/siyi_aruco"

# 2. Input: Đọc từ MediaMTX (Localhost) để tránh tranh chấp với Camera
# (MediaMTX đã kết nối với Camera rồi, Python chỉ việc lấy lại từ nó cho nhẹ)
CAMERA_URL = "rtsp://127.0.0.1:8554/my_camera" 

# Cổng nhận dữ liệu GPS (nếu có)
GPS_PORT = 5555

# Kích thước Stream Output: HD 720p
STREAM_W, STREAM_H = 1280, 720 

# --- CẤU HÌNH FFMPEG (Tối ưu cho Pi CM4/ARM64) ---
def get_ffmpeg_command(width, height, fps):
    return [
        'ffmpeg',
        '-y',
        '-f', 'rawvideo',
        '-vcodec', 'rawvideo',
        '-pix_fmt', 'bgr24',
        '-s', f'{width}x{height}', 
        '-r', str(fps),
        '-i', '-',
        '-c:v', 'libx264',
        '-preset', 'ultrafast',     # Nén siêu nhanh để giảm tải CPU Pi
        '-tune', 'zerolatency',     # Giảm độ trễ xuống thấp nhất
        '-profile:v', 'baseline',   # Bắt buộc để WebRTC chạy được
        '-pix_fmt', 'yuv420p',      # Bắt buộc để WebRTC chạy được
        
        # Bitrate 2000k là mức "Vàng" cho Wifi Pi (Nét mà không lag)
        '-b:v', '2000k',
        '-maxrate', '2500k',
        '-bufsize', '1000k',
        '-g', '30',                 # Keyframe mỗi 1s
        '-threads', '4',            # Tận dụng 4 nhân CPU của Pi CM4
        
        '-f', 'rtsp',
        RTSP_PUSH_URL
    ]

# --- GPS THREAD ---
running = True 
current_lat, current_lon = 0.0, 0.0

def gps_thread():
    global current_lat, current_lon
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("0.0.0.0", GPS_PORT))
        sock.settimeout(1.0)
    except: return
    while running:
        try:
            data, _ = sock.recvfrom(1024)
            _, _, lat, lon, _ = struct.unpack('qi3d', data)
            current_lat, current_lon = lat, lon
        except: pass

# --- MAIN ---
def main():
    global running
    print(f">>> Connecting to Input: {CAMERA_URL}")
    
    # --- PHẦN KẾT NỐI CAMERA (CÓ RETRY LOGIC) ---
    cap = None
    retry_count = 0
    
    while running:
        # Dùng GStreamer Pipeline để giảm độ trễ khi đọc từ localhost
        pipeline_in = (
            f"rtspsrc location={CAMERA_URL} latency=0 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! "
            "videoconvert ! appsink sync=false drop=true max-buffers=1"
        )
        
        try:
            cap = cv2.VideoCapture(pipeline_in, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                print("✅ Connected via GStreamer!")
                break 
        except: pass
        
        # Fallback: Nếu Gstreamer lỗi thì dùng API thường
        print("⚠️ GStreamer failed, trying default API...")
        cap = cv2.VideoCapture(CAMERA_URL)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        if cap.isOpened():
            print("✅ Connected via Standard API!")
            break
            
        print(f"Waiting for MediaMTX stream... (Retry {retry_count})")
        time.sleep(2) # Đợi 2s rồi thử lại
        retry_count += 1

    # Lấy kích thước gốc
    orig_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    orig_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    # Nếu không đọc được kích thước (do stream chưa init xong), gán mặc định HD
    if orig_w == 0: orig_w, orig_h = 1280, 720
    fps = 30
    
    print(f"✅ Input: {orig_w}x{orig_h} -> Output: {STREAM_W}x{STREAM_H}")

    # Khởi tạo FFmpeg
    ffmpeg_cmd = get_ffmpeg_command(STREAM_W, STREAM_H, fps)
    try:
        process = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)
    except Exception as e:
        print(f"❌ FFmpeg Error: {e}")
        return

    # Setup ArUco
    try:
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        parameters = aruco.DetectorParameters_create()
    except:
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
        parameters = aruco.DetectorParameters()
    
    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(aruco_dict, parameters)
    else:
        detector = None

    print(">>> Streaming Started! View at link: .../siyi_aruco")

    while True:
        ret, frame = cap.read()
        if not ret: 
            print("⚠️ Stream lost frame, skipping...")
            time.sleep(0.01)
            continue
        
        # 1. Xử lý ArUco trên ảnh gốc (để chính xác nhất)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if detector:
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                c = corners[i][0]
                cx, cy = int((c[0][0] + c[2][0]) / 2), int((c[0][1] + c[2][1]) / 2)
                cv2.circle(frame, (cx, cy), 10, (0, 0, 255), 2)
                cv2.putText(frame, f"ID:{ids[i][0]}", (cx, cy-15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # 2. Vẽ thông tin debug (Crosshair & GPS)
        cv2.drawMarker(frame, (orig_w//2, orig_h//2), (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
        cv2.putText(frame, "PI CM4 - ARUCO", (50, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.putText(frame, f"GPS: {current_lat:.5f}, {current_lon:.5f}", 
                    (50, orig_h - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 3. Resize về chuẩn HD (nếu input khác size) và đẩy đi
        if (orig_w != STREAM_W) or (orig_h != STREAM_H):
            frame_final = cv2.resize(frame, (STREAM_W, STREAM_H))
        else:
            frame_final = frame

        try:
            process.stdin.write(frame_final.tobytes())
        except: break

    running = False
    cap.release()
    process.stdin.close()
    process.wait()

if __name__ == "__main__":
    t = threading.Thread(target=gps_thread, daemon=True)
    t.start()
    main()
