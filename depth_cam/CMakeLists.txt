cmake_minimum_required(VERSION 3.3)

project(depth_cam CXX)

find_package(OpenCV REQUIRED)

set(TARGET depth_cam)


add_executable(${TARGET} main.cpp)
target_link_libraries(${TARGET} ${OpenCV_LIBS} realsense2)
