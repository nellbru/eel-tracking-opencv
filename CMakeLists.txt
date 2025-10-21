cmake_minimum_required(VERSION 4.0)
project(eel_tracking)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(eel_tracking eel_tracking.cpp)
target_link_libraries(eel_tracking ${OpenCV_LIBS})