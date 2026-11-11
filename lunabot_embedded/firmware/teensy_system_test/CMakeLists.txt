cmake_minimum_required(VERSION 3.20)
project(teensy_system_test LANGUAGES C CXX)

add_executable(teensy_system_test teensy_system_test.cpp interfaces.cpp)
make_hex(teensy_system_test)
target_link_libraries(teensy_system_test teensyduino::teensy4_core lunabot_lib teensyduino::arduino_stepper-src)
target_include_directories(teensy_system_test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
