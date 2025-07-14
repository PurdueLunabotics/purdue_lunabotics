cmake_minimum_required(VERSION 3.8)
project(lunabot_embedded)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED) # build tool for C++

# add to the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


# set a c++ executable to build. Optionally, include further cpp source files after the main one.
add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# 'install' this executable/node, so that it can be run using "ros2 run <package> <node>"
install(TARGETS 
        talker
        DESTINATION lib/${PROJECT_NAME})

# same thing, for listener
add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
        talker
        listener
        DESTINATION lib/${PROJECT_NAME})


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
