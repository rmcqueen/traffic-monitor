cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
project(traffic-monitor)

find_package(OpenCV)
include_directories(include)
set(SOURCE_FILES
        src/main.cpp
        src/Blob.cpp
        src/Transform.cpp
        include/Blob.h
        include/Transform.h)
add_executable(traffic-monitor ${SOURCE_FILES})
target_link_libraries(traffic-monitor ${OpenCV_LIBS})
