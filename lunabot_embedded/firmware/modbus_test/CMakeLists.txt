cmake_minimum_required(VERSION 3.20)
project(modbus_test LANGUAGES C CXX)

add_executable(modbus_test modbus_test.cpp)
make_hex(modbus_test)
target_link_libraries(modbus_test teensyduino::teensy4_core lunabot_lib)
