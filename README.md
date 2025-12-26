# pi_git_backup

Vì Pi CM4 chạy kiến trúc ARM64 và được cài OS Ubuntu Server (không màn hình). Dưới đây là "Cẩm nang cài đặt SIYI WebRTC Streamer trên CM4" từ A-Z.

## 1. CẤU HÌNH MẠNG (QUAN TRỌNG NHẤT)

Pi CM4 cần 2 đường mạng:

Ethernet (eth0): IP Tĩnh để nói chuyện với Camera (192.168.168.x).

Wifi (wlan0): IP để máy tính Ground Station kết nối vào xem.

Bước 1.1: Kiểm tra tên card mạng
```bash
ip link show
```
Ghi nhớ tên card: thường là eth0 và wlan0.

Bước 1.2: Cấu hình Netplan
Ubuntu Server dùng Netplan. Hãy sửa file cấu hình:

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

(Nếu tên file khác, cứ mở file có đuôi .yaml trong thư mục đó).

Xóa hết nội dung cũ và dán nội dung này vào (Sửa SSID và PASSWORD wifi của bạn):

```bash
network:
  ethernets:
    eth0:
      dhcp4: false
      addresses:
        - 192.168.168.30/24  # IP tĩnh của Pi để nối Camera
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "Tên_Wifi_Của_Bạn":
          password: "Mật_khẩu_Wifi"
      optional: true
  version: 2
```

Bước 1.3: Áp dụng

```bash
sudo netplan apply
```  

Sau đó gõ

```bash
ip a
```

để chắc chắn eth0 đã có IP .30 và wlan0 đã có IP Wifi.

## 2. CÀI ĐẶT MÔI TRƯỜNG
Do là bản Server rút gọn, ta cần cài thủ công các thư viện xử lý ảnh và video.

Bước 2.1: Cập nhật hệ thống

```bash
sudo apt update && sudo apt upgrade -y
```

Bước 2.2: Cài FFmpeg, GStreamer và thư viện bổ trợ

```bash
sudo apt install -y python3-pip ffmpeg libsm6 libxext6 libgl1-mesa-glx
sudo apt install -y libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools
```

Bước 2.3: Cài thư viện Python
Ta dùng bản headless (không có GUI) cho nhẹ Pi.

```bash
pip install numpy opencv-python-headless opencv-contrib-python-headless
```

## 3. CÀI ĐẶT MEDIAMTX (BẢN CHO PI/ARM64)
Lưu ý: Bạn không thể copy file mediamtx từ PC sang vì PC là kiến trúc x86, còn Pi là ARM64. Phải tải bản mới.

Bước 3.1: Tải về

```bash
cd ~
wget https://github.com/bluenviron/mediamtx/releases/download/v1.15.5/mediamtx_v1.15.5_linux_arm64.tar.gz
tar -xvzf mediamtx_v1.15.5_linux_arm64.tar.gz
mv mediamtx mediamtx_server
rm mediamtx_v1.15.5_linux_arm64.tar.gz LICENSE
```

Bước 3.2: Tạo file cấu hình mediamtx.yml
```bash
nano mediamtx.yml
```

Sửa phần nội dung paths như ở dưới

```bash
paths:
  # Cấu hình luồng Camera SIYI (Input Proxy)
  my_camera:
    source: rtsp://192.168.168.14:8554/main.264
    rtspTransport: tcp
    sourceOnDemand: yes
```

Bước 3.3. Cấu hình Hệ điều hành & Kích hoạt GPU (Quan trọng nhất)
Ubuntu Server mặc định không bật các tính năng phần cứng của Pi.

Sửa file cấu hình Boot:

```bash
sudo nano /boot/firmware/config.txt
```

Thêm các dòng sau vào cuối file để cấp RAM cho GPU và bật Camera:

```bash
[all]
# Cấp phát RAM cho GPU để chạy Encoder (Bắt buộc)
gpu_mem=128
# Bật firmware mở rộng
start_x=1
```

Load driver Video4Linux2 (V4L2):

```bash
sudo nano /etc/modules

Thêm dòng này vào cuối file:

bcm2835-v4l2
```
Khởi động lại:

```bash
sudo reboot
```

Kiểm tra sau khi khởi động:
Chạy lệnh này để chắc chắn GPU đã sẵn sàng:

```bash
ffmpeg -encoders | grep v4l2
```

Kết quả phải có dòng: V....D h264_v4l2m2m

Bước 3.4: Check cam
Mở terminal

```bash
./mediamtx_server
```
trên web

```
http://<IP_cua_PI>:8889/my_camera/
```
Đến đây đã kết nối xong, nếu chỉ để kết nối Pi và Cam thì đã hoàn thành.

## 4.Chạy toàn bộ hệ thống
**4.1. Đảm bảo hệ thống ở trạng thái tốt nhất trước khi cất cánh**
Chạy benchmark pre-flight
```bash
cd benchmark
./ultimate_audit.py
```
Nếu thấy lỗi cần fix thì làm theo hướng dẫn

**4.2. Khởi chạy cam Siyi A8**
chạy mediamtx
```bash
cd mediamtx
./mediamtx
```

**4.3. Chạy script detect Aruco**
Mở 1 terminal khác
```bash
cd stream_cam
```

Nếu chỉ cần detect Aruco thì chạy
```bash
python3 main_pi_stream.py
```

Nếu cần cả thêm chức năng tracking thì chạy
```bash
cd stream_cam/tracking_gimbal
python3 pi_tracking.py
```

xem hiển thị tại trình duyệt web
```bash
http://dia_chi_IP_cua_PI:8889/siyi_aruco/
```
Chạy xong tracking muốn cam về vị trí 0 thì chạy file
```bash
python3 reset_center.py
```
