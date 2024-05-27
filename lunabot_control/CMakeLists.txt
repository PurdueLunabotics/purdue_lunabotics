cmake_minimum_required(VERSION 3.5)
project(lunabot_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

ament_export_dependencies(rclcpp tf2_ros geometry_msgs nav2_costmap_2d nav_msgs)
ament_export_dependencies(EIGEN3)

install(DIRECTORY include/
  DESTINATION include)
ament_export_include_directories(include)

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lunabot_control
#   CATKIN_DEPENDS roscpp tf2_ros geometry_msgs costmap_2d nav_msgs swri_profiler
#   DEPENDS EIGEN3 
#  DEPENDS system_lib
#)

add_compile_options(-pg)
set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(include)

#include_directories(
# include
# ${catkin_INCLUDE_DIRS}
# ${EIGEN3_INCLUDE_DIRS}
#)

add_executable(${PROJECT_NAME}_mpc_node src/mpc_node.cpp src/mpc.cpp )
# set_target_properties(${PROJECT_NAME}_mpc_node PROPERTIES OUTPUT_NAME mpc_node PREFIX "")
# add_dependencies(${PROJECT_NAME}_mpc_node {${PROJECT_NAME}_EXPORTED_TARGETS})
ament_target_dependencies(${PROJECT_NAME}_mpc_node ${Eigen3_LIBRARIES} rclcpp geometry_msgs tf2_ros nav2_costmap_2d nav_msgs)

ament_package()

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lunabot_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
