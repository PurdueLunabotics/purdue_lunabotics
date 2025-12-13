cmake_minimum_required(VERSION 3.8)
project(lunabot_embedded)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED) # build tool for C++

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lunabot_msgs REQUIRED)

add_executable(teensy_driver_node
  src/teensy_driver_node.cpp
  src/hid.c
  src/pb_common.c 
  src/pb_encode.c 
  src/pb_decode.c 
  src/RobotMsgs.pb.c) 

target_link_libraries(teensy_driver_node usb)

target_include_directories(teensy_driver_node
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(teensy_driver_node rclcpp std_msgs lunabot_msgs )

install(TARGETS
  teensy_driver_node
  DESTINATION lib/${PROJECT_NAME})

# add to the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
