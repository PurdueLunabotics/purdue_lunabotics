cmake_minimum_required(VERSION 3.20)
project(teensy_main LANGUAGES C CXX)

add_executable(teensy_main teensy_main.cpp interfaces.cpp robot.cpp)
make_hex(teensy_main)
target_link_libraries(teensy_main teensyduino::teensy4_core lunabot_lib)
target_include_directories(teensy_main PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
