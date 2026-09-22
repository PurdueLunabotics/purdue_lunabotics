cmake_minimum_required(VERSION 3.20)
project(teensy_mini LANGUAGES C CXX)

add_executable(teensy_mini teensy_mini.cpp interfaces.cpp robot.cpp)
make_hex(teensy_mini)
target_link_libraries(teensy_mini teensyduino::teensy4_core lunabot_lib)
target_include_directories(teensy_mini PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
