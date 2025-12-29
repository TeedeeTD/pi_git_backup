#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>
#include <cmath>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <chrono>
#include <iomanip> 
#include <sstream>

// --- CẤU HÌNH ---
#define FIXED_FPS 20       
#define FRAME_TIME_MS (1000.0 / FIXED_FPS)
#define RTSP_PUSH_URL "rtsp://localhost:8554/siyi_aruco"
#define CAMERA_URL "rtsp://127.0.0.1:8554/my_camera"
#define STREAM_W 1280
#define STREAM_H 720

// Config Algo
#define DETECT_SCALE 1   
#define SKIP_FRAME 0       
#define SIYI_IP "192.168.168.14"
#define SIYI_PORT 37260
#define KP_YAW 0.12
#define KP_PITCH 0.12
#define DEADZONE 15

using namespace std;
using namespace cv;

// --- HÀM TẠO TÊN FILE LOG ---
string getLogFilename() {
    auto now = chrono::system_clock::to_time_t(chrono::system_clock::now());
    stringstream ss;
    ss << "pi_log_" << put_time(localtime(&now), "%Y%m%d_%H%M%S") << ".csv";
    return ss.str();
}

// ==========================================
// 1. CLASS BUFFERLESS CAPTURE
// ==========================================
class BufferlessCapture {
private:
    VideoCapture cap;
    Mat latest_frame;
    mutex mtx;
    thread t;
    atomic<bool> running;
    atomic<bool> has_frame;

    void update() {
        while (running) {
            Mat temp;
            if (cap.read(temp)) {
                lock_guard<mutex> lock(mtx);
                if (!temp.empty()) {
                    temp.copyTo(latest_frame);
                    has_frame = true;
                }
            } else {
                this_thread::sleep_for(chrono::milliseconds(10));
            }
        }
    }

public:
    BufferlessCapture(string url) {
        cap.open(url);
        cap.set(CAP_PROP_BUFFERSIZE, 1); 
        running = true;
        has_frame = false;
        if (cap.isOpened()) {
            cout << "✅ Camera Thread Started!" << endl;
            t = thread(&BufferlessCapture::update, this);
        } else {
            cerr << "❌ Failed to open camera!" << endl;
        }
    }

    ~BufferlessCapture() {
        running = false;
        if (t.joinable()) t.join();
        cap.release();
    }

    bool read(Mat& out) {
        if (!has_frame) return false;
        lock_guard<mutex> lock(mtx); 
        latest_frame.copyTo(out);
        return true;
    }
    
    bool isOpened() { return cap.isOpened(); }
};

// ==========================================
// 2. CLASS SIYI GIMBAL
// ==========================================
class SiyiGimbal {
private:
    int sock;
    struct sockaddr_in servaddr;
    uint16_t seq;
    chrono::steady_clock::time_point last_sent;

    uint16_t crc16(const vector<uint8_t>& data) {
        uint16_t crc = 0;
        for (uint8_t byte : data) {
            crc ^= (uint16_t)byte << 8;
            for (int i = 0; i < 8; i++) {
                if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
                else crc = crc << 1;
            }
        }
        return crc;
    }

public:
    SiyiGimbal() : seq(0) {
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(SIYI_PORT);
        servaddr.sin_addr.s_addr = inet_addr(SIYI_IP);
        last_sent = chrono::steady_clock::now();
    }
    ~SiyiGimbal() { close(sock); }

    void rotate(int yaw, int pitch) {
        auto now = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(now - last_sent).count() < 40) return; 
        last_sent = now;

        yaw = max(-100, min(100, yaw));
        pitch = max(-100, min(100, pitch));
        seq++;

        vector<uint8_t> payload = {(uint8_t)yaw, (uint8_t)pitch};
        vector<uint8_t> msg = {0x55, 0x66, 0x01}; 
        uint16_t len = payload.size();
        msg.push_back(len & 0xFF); msg.push_back(len >> 8);
        msg.push_back(seq & 0xFF); msg.push_back(seq >> 8);
        msg.push_back(0x07); 
        msg.insert(msg.end(), payload.begin(), payload.end());
        uint16_t crc = crc16(msg);
        msg.push_back(crc & 0xFF); msg.push_back(crc >> 8);
        sendto(sock, msg.data(), msg.size(), 0, (const struct sockaddr *)&servaddr, sizeof(servaddr));
    }
    
    void center() {
        seq++;
        vector<uint8_t> msg = {0x55, 0x66, 0x01, 0x01, 0x00};
        msg.push_back(seq & 0xFF); msg.push_back(seq >> 8); 
        msg.push_back(0x08); 
        msg.push_back(0x01); 
        uint16_t crc = crc16(msg);
        msg.push_back(crc & 0xFF); msg.push_back(crc >> 8);
        sendto(sock, msg.data(), msg.size(), 0, (const struct sockaddr *)&servaddr, sizeof(servaddr));
    }
};

// ==========================================
// 3. MAIN (PRINT LOOP TIME)
// ==========================================
int main() {
    cout << ">>> C++ TRACKING V3.3 (CONSOLE PRINTING ENABLED)..." << endl;

    string logFileName = getLogFilename();
    ofstream csvFile(logFileName);
    if (csvFile.is_open()) {
        csvFile << "Time_ms,Algo_ms,E2E_ms,FPS,ErrX,ErrY,Yaw_Cmd,Pitch_Cmd\n";
        cout << "📝 Logging to: " << logFileName << endl;
    }

    BufferlessCapture cap(CAMERA_URL);
    if (!cap.isOpened()) return -1;
    this_thread::sleep_for(chrono::seconds(1));

    string ffmpeg_cmd = "ffmpeg -y -f rawvideo -vcodec rawvideo -pix_fmt bgr24 -s 1280x720 "
                        "-r " + to_string(FIXED_FPS) + " -i - "
                        "-c:v h264_v4l2m2m -b:v 2000k -pix_fmt yuv420p "
                        "-g " + to_string(FIXED_FPS) + " "
                        "-bufsize 1000k -f rtsp " + string(RTSP_PUSH_URL);
    
    FILE* ffmpeg_pipe = popen(ffmpeg_cmd.c_str(), "w");
    if (!ffmpeg_pipe) return -1;

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_1000);
    Ptr<aruco::DetectorParameters> params = aruco::DetectorParameters::create();
    params->adaptiveThreshWinSizeStep = 20;

    SiyiGimbal gimbal;
    gimbal.center();

    Mat frame, small_frame, gray;
    vector<int> ids;
    vector<vector<Point2f>> corners;
    vector<vector<Point2f>> last_corners;
    vector<int> last_ids;
    
    int frame_count = 0;
    bool is_tracking = false;
    int last_yaw = 0, last_pitch = 0;
    int err_x = 0, err_y = 0;

    // Biến đo thời gian
    double algo_ms = 0.0, e2e_ms = 0.0, fps_real = 0.0;
    auto last_fps_check = chrono::high_resolution_clock::now();
    auto start_program_time = chrono::high_resolution_clock::now();
    int fps_counter = 0;

    cout << ">>> SYSTEM LIVE!" << endl;
    cout << "----------------------------------------------------------------" << endl;
    cout << "| Frame | Algo(ms) | E2E(ms) | Loop(ms) | Wait(ms) | FPS     |" << endl;
    cout << "----------------------------------------------------------------" << endl;

    while (true) {
        // [1] Bắt đầu Loop
        auto t_loop_start = chrono::high_resolution_clock::now();
        frame_count++;

        if (!cap.read(frame)) {
            this_thread::sleep_for(chrono::milliseconds(1));
            continue;
        }
        auto t_frame_ready = chrono::high_resolution_clock::now();
        auto t_algo_start = chrono::high_resolution_clock::now();
        
        // [2] Detect & Control
        bool should_detect = (frame_count % (SKIP_FRAME + 1) == 0);
        if (should_detect) {
            resize(frame, small_frame, Size(), DETECT_SCALE, DETECT_SCALE, INTER_NEAREST);
            cvtColor(small_frame, gray, COLOR_BGR2GRAY);
            aruco::detectMarkers(gray, dictionary, corners, ids, params);
            
            if (!ids.empty()) {
                float scale_factor = 1.0 / DETECT_SCALE;
                for(auto& corner_vec : corners) {
                    for(auto& point : corner_vec) point *= scale_factor;
                }
                last_corners = corners; last_ids = ids; is_tracking = true;
            } else {
                is_tracking = false;
            }
        }

        if (is_tracking && !last_corners.empty()) {
            Point2f center = last_corners[0][0] + last_corners[0][2];
            center *= 0.5;
            err_x = (int)center.x - (STREAM_W / 2); 
            err_y = (int)center.y - (STREAM_H / 2);

            if (abs(err_x) > DEADZONE) last_yaw = (int)(err_x * KP_YAW); else last_yaw = 0;
            if (abs(err_y) > DEADZONE) last_pitch = (int)(-err_y * KP_PITCH); else last_pitch = 0;

            aruco::drawDetectedMarkers(frame, last_corners, last_ids);
            line(frame, Point(STREAM_W/2, STREAM_H/2), Point((int)center.x, (int)center.y), Scalar(0,0,255), 2);
        } else {
            last_yaw = 0; last_pitch = 0; err_x = 0; err_y = 0;
        }

        gimbal.rotate(last_yaw, last_pitch);

        auto t_algo_end = chrono::high_resolution_clock::now();
        algo_ms = chrono::duration<double, milli>(t_algo_end - t_algo_start).count();
        e2e_ms = chrono::duration<double, milli>(t_algo_end - t_frame_ready).count();

        // [3] Ghi Log CSV
        if (csvFile.is_open()) {
            double current_time_ms = chrono::duration<double, milli>(t_algo_end - start_program_time).count();
            csvFile << fixed << setprecision(1) << current_time_ms << ","
                    << setprecision(2) << algo_ms << "," << e2e_ms << "," << fps_real << ","
                    << err_x << "," << err_y << "," << last_yaw << "," << last_pitch << "\n";
        }

        // [4] Push Stream
        fwrite(frame.data, 1, STREAM_W * STREAM_H * 3, ffmpeg_pipe);

        // [5] Tính toán & Ngủ
        auto t_loop_end = chrono::high_resolution_clock::now();
        
        // Tổng thời gian máy bận rộn (Active time)
        double loop_busy_ms = chrono::duration<double, milli>(t_loop_end - t_loop_start).count();
        double wait_ms = FRAME_TIME_MS - loop_busy_ms;
        
        // --- [MỚI] IN RA CONSOLE CHO TỪNG FRAME ---
        cout << "| " << setw(5) << frame_count 
             << " | " << setw(8) << fixed << setprecision(1) << algo_ms 
             << " | " << setw(7) << e2e_ms 
             << " | " << setw(8) << loop_busy_ms 
             << " | " << setw(8) << (wait_ms > 0 ? wait_ms : 0) 
             << " | " << setw(5) << setprecision(1) << fps_real << " |" 
             << "\r" << flush; // \r để in đè lên dòng cũ cho đỡ trôi màn hình
             
        // Nếu bạn muốn in dòng mới liên tục (log trôi) thì thay "\r" bằng endl

        if (wait_ms > 0) this_thread::sleep_for(chrono::milliseconds((int)wait_ms));

        fps_counter++;
        if (fps_counter >= 10) {
            auto now = chrono::high_resolution_clock::now();
            fps_real = fps_counter / chrono::duration<double>(now - last_fps_check).count();
            last_fps_check = now; fps_counter = 0;
        }
    }

    if (csvFile.is_open()) csvFile.close();
    pclose(ffmpeg_pipe);
    return 0;
}
