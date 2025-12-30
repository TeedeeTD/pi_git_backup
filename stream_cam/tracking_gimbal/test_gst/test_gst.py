import cv2
import os

# Bật debug log
os.environ["OPENCV_VIDEOIO_DEBUG"] = "1"

src = "rtsp://127.0.0.1:8554/my_camera"

# Danh sách các kiểu Pipeline nghi ngờ
pipelines = [
    # Kiểu 1: Cú pháp gọn (Khuyên dùng)
    f"rtspsrc location={src} latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! queue ! videoconvert ! video/x-raw,format=BGR ! appsink sync=false drop=true",
    
    # Kiểu 2: Thêm (string)
    f"rtspsrc location={src} latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! queue ! videoconvert ! video/x-raw,format=(string)BGR ! appsink sync=false drop=true",
    
    # Kiểu 3: Dùng decodebin (Tự động)
    f"rtspsrc location={src} latency=0 ! rtph264depay ! h264parse ! decodebin ! queue ! videoconvert ! video/x-raw,format=BGR ! appsink sync=false drop=true"
    #Type 4
    f"rtspsrc location={src} latency=0 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! "
            "queue max-size-buffers=1 ! "
            "videoconvert ! video/x-raw,format=BGR ! " 
            "appsink sync=false drop=true"
]

for i, p in enumerate(pipelines):
    print(f"\n--- TESTING PIPELINE {i+1} ---")
    cap = cv2.VideoCapture(p, cv2.CAP_GSTREAMER)
    if cap.isOpened():
        print("✅ SUCCESS!")
        ret, frame = cap.read()
        print(f"Frame Size: {frame.shape if ret else 'None'}")
        cap.release()
        break
    else:
        print("❌ FAILED.")
