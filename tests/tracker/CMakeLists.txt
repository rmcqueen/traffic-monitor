cmake_minimum_required(VERSION 3.1)
project(test_tracker)

set(SOURCE_FILES main.cpp)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
add_executable(test_tracker ${SOURCE_FILES})

target_link_libraries(test_tracker lib-traffic-monitor
        core-traffic-monitor
        gtest
        gmock
        ${OpenCV_LIBS})

enable_testing()
add_test(NAME test_tracker COMMAND test_tracker)