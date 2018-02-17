cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
project(traffic-monitor)

find_package(OpenCV REQUIRED)

include_directories(include)
include_directories(tests)
include_directories(src)

set(SOURCE_FILES
        src/main.cpp
        src/Blob.cpp
        src/Transform.cpp
        include/Blob.h
        include/Transform.h)

add_subdirectory(tests)

add_executable(traffic-monitor ${SOURCE_FILES})

target_link_libraries(traffic-monitor ${OpenCV_LIBS})
