cmake_minimum_required(VERSION 3.20)
project(hid_test LANGUAGES C CXX)

add_executable(hid_test hid_test.cpp)
make_hex(hid_test)
target_link_libraries(hid_test teensyduino::teensy4_core)
