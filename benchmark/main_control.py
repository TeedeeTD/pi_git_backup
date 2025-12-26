import time
import os
import gc
# import cv2 # Thư viện xử lý ảnh

# TẮT GC ĐỂ TRÁNH GIẬT LAG BẤT THƯỜNG
gc.disable() 

def main_control_loop():
    print(f"PID: {os.getpid()} - Chạy trên Core được chỉ định (Core 3)")
    
    frame_count = 0
    
    try:
        while True:
            # 1. BẮT ĐẦU ĐỒNG HỒ
            start_t = time.perf_counter()
            
            # 2. XỬ LÝ CHÍNH (SENSE -> THINK -> ACT)
            # data = sensor.read()
            # control_signal = pid_process(data)
            # motor.write(control_signal)
            
            time.sleep(0.005) # Giả lập công việc mất 5ms
            
            # 3. DỌN RÁC THỦ CÔNG (Mỗi 1000 frame dọn 1 lần để kiểm soát)
            if frame_count % 1000 == 0:
                gc.collect()
            
            # 4. KẾT THÚC ĐỒNG HỒ
            end_t = time.perf_counter()
            loop_time = (end_t - start_t) * 1000 # đổi ra ms
            
            # 5. TỰ GIÁM SÁT (HEARTBEAT LOG)
            # In ra mỗi 100 vòng lặp (khoảng 1 giây)
            if frame_count % 100 == 0:
                status = "OK" if loop_time < 20 else "LAG"
                print(f"[Core 3] Loop: {loop_time:.3f} ms | Status: {status}")
            
            frame_count += 1
            
    except KeyboardInterrupt:
        print("Dừng hệ thống.")
        gc.enable()

if __name__ == "__main__":
    main_control_loop()
