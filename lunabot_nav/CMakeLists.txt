cmake_minimum_required(VERSION 3.0.2)
project(lunabot_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  geometry_msgs
  costmap_2d
)

catkin_python_setup()

find_package(Eigen3 REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lunabot_nav
   CATKIN_DEPENDS roscpp tf2_ros geometry_msgs costmap_2d nav_msgs
   DEPENDS EIGEN3 
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/lunabot_nav.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_mpc_node nodes/mpc_node.cpp src/mpc.cpp )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_mpc_node PROPERTIES OUTPUT_NAME mpc_node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_mpc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_mpc_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

add_executable(${PROJECT_NAME}_costmap_node nodes/costmap_node.cpp)
set_target_properties(${PROJECT_NAME}_costmap_node PROPERTIES OUTPUT_NAME costmap_node PREFIX "")
add_dependencies(${PROJECT_NAME}_costmap_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_costmap_node
 ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lunabot_nav.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# Tests
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)
  find_package(rostest REQUIRED)
  add_rostest(test/test_global_planner.test)
  catkin_add_nosetests(test)
endif()
