cmake_minimum_required(VERSION 3.1)
project(test_blob)

set(SOURCE_FILES
        BlobTest.cpp main.cpp)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
add_executable(test_blob ${SOURCE_FILES})

target_link_libraries(test_blob lib-traffic-monitor
        core-traffic-monitor
        gtest
        gmock
        ${OpenCV_LIBS})
enable_testing()
add_test(NAME test_blob COMMAND test_blob)