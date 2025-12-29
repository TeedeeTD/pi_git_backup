cmake_minimum_required(VERSION 3.0)
project(PiTracking)

# Tìm OpenCV (C++ Version)
find_package(OpenCV REQUIRED)

# Thêm luồng (Threading)
find_package(Threads REQUIRED)

# Cấu hình file thực thi
add_executable(pi_track main.cpp)

# Link thư viện
target_link_libraries(pi_track ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})
