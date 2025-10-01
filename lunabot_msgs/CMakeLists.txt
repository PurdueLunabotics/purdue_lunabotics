cmake_minimum_required(VERSION 3.8)
project(lunabot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# add to the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED) # required for building messages
find_package(std_msgs REQUIRED)

# Generate message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotEffort.msg"
  "msg/RobotSensors.msg"
  "msg/RobotErrors.msg"
  DEPENDENCIES std_msgs
)
# Add any packages that above messages depend on in a list next to DEPENDENCIES.

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
