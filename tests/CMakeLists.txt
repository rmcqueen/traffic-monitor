cmake_minimum_required(VERSION 3.1)
project(test_traffic_monitor)

find_package(GTest REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})
include_directories(lib)
include_directories(include)

set(SOURCE_FILES main.cpp)
add_executable(test_traffic_monitor ${SOURCE_FILES})

add_subdirectory(blob)
add_subdirectory(transform)

target_link_libraries(test_traffic_monitor ${GTEST_LIBRARIES} pthread)
target_link_libraries(test_traffic_monitor ${OpenCV_LIBS})
enable_testing()
add_test(NAME test_traffic_monitor COMMAND test_traffic_monitor)