cmake_minimum_required (VERSION 2.8)

project(undistorted-line-recognition)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(undistorted-line-recognition undistorted-line-recognition.cpp)

target_link_libraries(undistorted-line-recognition ${OpenCV_LIBS})