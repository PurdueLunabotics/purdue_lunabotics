cmake_minimum_required(VERSION 3.14.4)
project(lunabot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(image_proc REQUIRED)
find_package(rtabmap_util REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rtabmap_msgs REQUIRED)


# Install launch files
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/)

# Install C++ files
install(TARGETS
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/apriltag_combine.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(bbox_mask_node src/bbox_mask_node.cpp)

ament_target_dependencies(bbox_mask_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  image_geometry
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
  rtabmap_msgs
)


# Link Libraries
target_link_libraries(bbox_mask_node ${OpenCV_LIBS})


# Install Programs
install(TARGETS
  bbox_mask_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS bbox_mask_node
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

ament_package()
