cmake_minimum_required(VERSION 3.3)

project(cam_calibration CXX)

find_package(OpenCV REQUIRED)

set(TARGET_2D calibrate2d)
set(TARGET_2D_VALIDATE calibrate2d_validate)

add_executable(${TARGET_2D} calibrate2d.cpp)
target_link_libraries(${TARGET_2D} ${OpenCV_LIBS})

add_executable(${TARGET_2D_VALIDATE} calibrate2d_validate.cpp)
target_link_libraries(${TARGET_2D_VALIDATE} ${OpenCV_LIBS})