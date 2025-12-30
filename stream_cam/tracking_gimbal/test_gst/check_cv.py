import cv2
print(f"OpenCV Version: {cv2.__version__}")
build_info = cv2.getBuildInformation()
if "GStreamer:                    NO" in build_info:
    print("❌ LỖI LỚN: Bản OpenCV này KHÔNG hỗ trợ GStreamer!")
    print("👉 Giải pháp: Bạn đang dùng bản 'pip install'. Hãy dùng bản của apt: 'sudo apt install python3-opencv'")
else:
    print("✅ OpenCV có hỗ trợ GStreamer. Vấn đề nằm ở cú pháp Pipeline.")
