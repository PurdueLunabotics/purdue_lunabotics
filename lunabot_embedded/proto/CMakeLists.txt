cmake_minimum_required(VERSION 3.5)
project(proto C)

include(FetchContent)
FetchContent_Declare(
  NANOPB_CONTENT
  GIT_REPOSITORY https://github.com/nanopb/nanopb.git
  GIT_TAG        0.4.92
)
FetchContent_MakeAvailable(NANOPB_CONTENT)

set(CMAKE_MODULE_PATH ${nanopb_content_SOURCE_DIR}/extra)
find_package(Nanopb REQUIRED)

nanopb_generate_cpp(TARGET proto RobotMsgs.proto)
