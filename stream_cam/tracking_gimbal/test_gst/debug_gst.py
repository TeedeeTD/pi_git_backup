import cv2
import os
import time

# --- BẬT LOG GSTREAMER ---
# Mức 3 = Warning/Error. Mức 4 = Info. Mức 5 = Debug (Rất nhiều chữ)
os.environ["GST_DEBUG"] = "3" 

print(">>> INIT OPENCV GSTREAMER DEBUG...")

# Pipeline chuẩn nhất (dựa trên lệnh gst-launch đã chạy được của bạn)
# Lưu ý: appsink cần drop=true để không bị tràn bộ nhớ nếu xử lý chậm
pipeline = (
    "rtspsrc location=rtsp://127.0.0.1:8554/my_camera latency=0 ! "
    "rtph264depay ! h264parse ! avdec_h264 ! "
    "videoconvert ! video/x-raw,format=BGR ! "
    "appsink sync=false drop=true"
)

print(f"Pipeline: {pipeline}")

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if cap.isOpened():
    print("✅ SUCCESS! GStreamer is working via OpenCV.")
    ret, frame = cap.read()
    if ret:
        print(f"Frame received! Size: {frame.shape}")
    else:
        print("❌ Opened but no frame.")
else:
    print("❌ FAILED to open pipeline.")
    print("👉 HÃY ĐỌC LOG MÀU ĐỎ/VÀNG Ở TRÊN ĐỂ TÌM NGUYÊN NHÂN")

cap.release()
