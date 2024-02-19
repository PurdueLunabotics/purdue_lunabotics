cmake_minimum_required(VERSION 3.0.2)
project(lunabot_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  geometry_msgs
  costmap_2d
  swri_profiler
)

find_package(Eigen3 REQUIRED)
find_package(OpenMP)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lunabot_control
   CATKIN_DEPENDS roscpp tf2_ros geometry_msgs costmap_2d nav_msgs swri_profiler
   DEPENDS EIGEN3 
#  DEPENDS system_lib
)

add_compile_options(-pg)
set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_mpc_node src/mpc_node.cpp src/mpc.cpp )
set_target_properties(${PROJECT_NAME}_mpc_node PROPERTIES OUTPUT_NAME mpc_node PREFIX "")
add_dependencies(${PROJECT_NAME}_mpc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_mpc_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)


## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lunabot_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
