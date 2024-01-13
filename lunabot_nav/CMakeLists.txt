cmake_minimum_required(VERSION 3.0.2)
project(lunabot_nav)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lunabot_nav
   CATKIN_DEPENDS 
#  DEPENDS system_lib
)

add_compile_options(-pg)
set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)

include_directories(
 ${catkin_INCLUDE_DIRS}
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
  #roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)
  find_package(rostest REQUIRED)
  #add_rostest(test/test_global_planner.test)
  #catkin_add_nosetests(test)
endif()
