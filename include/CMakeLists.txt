cmake_minimum_required(VERSION 3.1)

set(SOURCE_FILES Blob.hpp
        Tracker.hpp
        Transform.hpp
        BackgroundSubtractor.hpp
        AppConfig.hpp)

add_library(lib-traffic-monitor INTERFACE)