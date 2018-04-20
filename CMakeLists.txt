cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
project(traffic-monitor)
find_package(OpenCV REQUIRED)

include_directories(include)
include_directories(tests)
include_directories(src)

set(SOURCE_FILES
        src/AppConfig.cpp
        src/Blob.cpp
        src/Transform.cpp
        include/Blob.hpp
        include/Transform.hpp
        src/Tracker.cpp
        include/Tracker.hpp
        src/BackgroundSubstractor.cpp
        include/BackgroundSubtractor.hpp
        include/AppConfig.hpp
        src/AppConfig.cpp
        src/main.cpp)

add_subdirectory(tests)
add_subdirectory(src)
add_subdirectory(include)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
add_executable(traffic-monitor ${SOURCE_FILES})

target_link_libraries(traffic-monitor lib-traffic-monitor
        core-traffic-monitor)
target_link_libraries(traffic-monitor gtest)
target_link_libraries(traffic-monitor ${OpenCV_LIBS})
