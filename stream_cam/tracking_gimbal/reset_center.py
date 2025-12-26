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

def send_center_command(sock):
    """Gửi lệnh 0x08: GIMBAL CENTER"""
    header = b'\x55\x66' # STX
    ctrl = b'\x01'       # Need ACK
    cmd_id = b'\x08'     # 0x08 Center
    payload = b'\x01'    # Value 1
    
    length = struct.pack('<H', len(payload))
    seq = struct.pack('<H', 0)
    
    msg = header + ctrl + length + seq + cmd_id + payload
    full_msg = msg + crc16_siyi(msg)
    
    print(f"📡 Gửi lệnh CENTER tới {SIYI_IP}...")
    sock.sendto(full_msg, (SIYI_IP, SIYI_PORT))

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Gửi 3 lần cho chắc ăn
        for i in range(3):
            send_center_command(sock)
            time.sleep(0.1)
        print("✅ Đã gửi lệnh Reset Gimbal.")
    except Exception as e:
        print(f"❌ Lỗi: {e}")

if __name__ == "__main__":
    main()
