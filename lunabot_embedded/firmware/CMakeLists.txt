include(FetchContent)

FetchContent_Declare(
    teensy_package
    URL https://www.pjrc.com/teensy/td_162/teensy-package-1.62.0.tar.zst
    URL_HASH SHA256=472d0a66708e366bf447c7870032edc549a496d671d651d51e2d3c3f28baffb9
    DOWNLOAD_EXTRACT_TIMESTAMP true
)
FetchContent_MakeAvailable(teensy_package)

cmake_minimum_required(VERSION 3.20)
project(firmware LANGUAGES C CXX)

add_subdirectory(teensyduino)
add_subdirectory(hid_test)
