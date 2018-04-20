cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
project(core-traffic-monitor)

set(SOURCE_FILES Blob.cpp
        Tracker.cpp
        Transform.cpp
        AppConfig.cpp
        main.cpp
        BackgroundSubstractor.cpp)

add_library(core-traffic-monitor ${SOURCE_FILES})
target_include_directories(core-traffic-monitor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})