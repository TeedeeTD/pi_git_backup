import cv2
import os

# Bật log chi tiết
os.environ["GST_DEBUG"] = "3"

print(">>> TEST 1: VIDEOTESTSRC (Giả lập video)...")
# Pipeline đơn giản nhất: Tạo video cầu vồng -> Convert sang BGR -> Appsink
pipeline = "videotestsrc ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if cap.isOpened():
    print("✅ THÀNH CÔNG! OpenCV GStreamer hoạt động tốt.")
    ret, frame = cap.read()
    if ret:
        print(f"   Đã đọc được frame kích thước: {frame.shape}")
    else:
        print("   Nhưng không đọc được frame.")
else:
    print("❌ THẤT BẠI! OpenCV không thể gọi GStreamer.")
    print("   -> Khả năng cao OpenCV pip install bị thiếu file liên kết.")

cap.release()
