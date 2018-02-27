cmake_minimum_required(VERSION 3.1)
project(test_traffic_monitor)

find_package(GTest REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})
include_directories(lib)
include_directories(include)
include_directories(src)

set(SOURCE_FILES
        main.cpp
        blob/BlobTest.cpp)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
add_executable(test_traffic_monitor ${SOURCE_FILES})

add_subdirectory(blob)
add_subdirectory(transform)
add_subdirectory(tracker)

include_directories(data)

target_link_libraries(test_traffic_monitor ${GTEST_LIBRARIES} pthread)
target_link_libraries(test_traffic_monitor ${OpenCV_LIBS})
target_link_libraries(test_traffic_monitor lib-traffic-monitor)
target_link_libraries(test_traffic_monitor core-traffic-monitor)
enable_testing()
add_test(NAME test_traffic_monitor COMMAND test_traffic_monitor)