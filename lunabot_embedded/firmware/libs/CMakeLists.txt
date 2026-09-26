cmake_minimum_required(VERSION 3.20)
project(hid_test LANGUAGES C CXX)

add_library(lunabot_lib STATIC
    ADS1119.cpp
    Adafruit_I2CDevice.cpp
    Adafruit_PWMServoDriver.cpp
    ModbusLite.cpp
    Sabertooth.cpp
    StepperLib.cpp)
target_link_libraries(lunabot_lib teensyduino::teensy4_core teensyduino::FastLED teensyduino::SPI teensyduino::Wire teensyduino::Encoder teensyduino::Servo proto)
target_include_directories(lunabot_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
