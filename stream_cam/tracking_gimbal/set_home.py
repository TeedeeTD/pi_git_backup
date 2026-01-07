import socket
import struct
import time

# --- CẤU HÌNH ---
SIYI_IP = "192.168.168.14"
SIYI_PORT = 37260

def crc16_siyi(data):
    """Tính CRC-16 cho gói tin SIYI"""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000: crc = (crc << 1) ^ 0x1021
            else: crc = crc << 1
            crc &= 0xFFFF
    return struct.pack('<H', crc)

def set_gimbal_angle(sock, yaw_deg, pitch_deg):
    """
    Gửi lệnh 0x0E: Điều khiển góc quay Gimbal
    yaw_deg: Góc xoay ngang (VD: 0)
    pitch_deg: Góc ngẩng/cúi (VD: -90 là nhìn thẳng xuống đất)
    """
    header = b'\x55\x66' # STX
    ctrl = b'\x01'       # Need ACK
    cmd_id = b'\x0E'     # 0x0E là lệnh SET ANGLE
    
    # SIYI dùng đơn vị 0.1 độ. Ví dụ: -90 độ = -900
    yaw_val = int(yaw_deg * 10)
    pitch_val = int(pitch_deg * 10)
    
    # Đóng gói payload: 2 số nguyên 16-bit có dấu (signed short)
    # '<hh': Little-endian, short (yaw), short (pitch)
    payload = struct.pack('<hh', yaw_val, pitch_val)
    
    length = struct.pack('<H', len(payload))
    seq = struct.pack('<H', 0)
    
    msg = header + ctrl + length + seq + cmd_id + payload
    full_msg = msg + crc16_siyi(msg)
    
    print(f"📡 Gửi lệnh Pitch {pitch_deg}° tới {SIYI_IP}...")
    sock.sendto(full_msg, (SIYI_IP, SIYI_PORT))

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Cúi xuống 90 độ (Yaw giữ 0)
        # Gửi vài lần để đảm bảo nhận được lệnh (giao thức UDP có thể rớt gói)
        for i in range(3):
            set_gimbal_angle(sock, yaw_deg=0, pitch_deg=-90) 
            time.sleep(0.1)
            
        print("✅ Đã gửi lệnh cúi xuống -90 độ.")
    except Exception as e:
        print(f"❌ Lỗi: {e}")

if __name__ == "__main__":
    main()
