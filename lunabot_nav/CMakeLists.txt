cmake_minimum_required(VERSION 3.14.4)
project(lunabot_nav)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(dstar_node src/dstar_node.cpp src/dstar.cpp)
add_library(dijkstra_plugin SHARED src/dijkstra.cpp)
add_executable(nav2_bridge_node src/nav2_bridge.cpp)

ament_target_dependencies(dstar_node rclcpp std_msgs nav_msgs map_msgs)
ament_target_dependencies(dijkstra_plugin rclcpp std_msgs nav_msgs map_msgs tf2_ros nav2_core nav2_costmap_2d nav2_util)
ament_target_dependencies(nav2_bridge_node rclcpp std_msgs nav_msgs rclcpp_action nav2_msgs)

# Install files

pluginlib_export_plugin_description_file(nav2_core dijkstra_planner.xml)

install(FILES dijkstra_planner.xml
  DESTINATION share/${PROJECT_NAME}
)

# Link Libraries


# Install Programs
install(TARGETS
  dstar_node
  nav2_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS dijkstra_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

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

ament_export_libraries(dijkstra_plugin)

ament_package()
