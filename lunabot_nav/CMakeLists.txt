cmake_minimum_required(VERSION 3.0.2)
project(lunabot_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  geometry_msgs
  costmap_2d
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lunabot_nav
   CATKIN_DEPENDS roscpp tf2_ros geometry_msgs costmap_2d nav_msgs
#  DEPENDS system_lib
)

add_compile_options(-pg)
set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_costmap_node src/costmap_node.cpp)
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
  #add_rostest(test/test_global_planner.test)
  catkin_add_nosetests(test)
endif()
