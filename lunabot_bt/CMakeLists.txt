cmake_minimum_required(VERSION 3.0.2)
project(lunabot_bt)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  lunabot_msgs
)

find_package(PkgConfig)
pkg_check_modules(BLEPP REQUIRED libblepp)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lunabot_bt 
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs lunabot_msgs
  DEPENDS BLEPP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${BLEPP_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_scan_node src/scan_node.cpp)
set_target_properties(${PROJECT_NAME}_scan_node PROPERTIES OUTPUT_NAME scan_node PREFIX "")
add_dependencies(${PROJECT_NAME}_scan_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_scan_node
  ${catkin_LIBRARIES}
  ${BLEPP_LIBRARIES}
)

add_executable(${PROJECT_NAME}_scan_test src/scan_test.cpp)
set_target_properties(${PROJECT_NAME}_scan_test PROPERTIES OUTPUT_NAME scan_test PREFIX "")
add_dependencies(${PROJECT_NAME}_scan_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_scan_test
  ${catkin_LIBRARIES}
  ${BLEPP_LIBRARIES}
)
