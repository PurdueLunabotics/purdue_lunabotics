cmake_minimum_required(VERSION 3.0.2)
project(dummy_bot_description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS 
  gazebo_dev
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  luna_gazebo_ros_skid_steer_drive

  CATKIN_DEPENDS 
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(luna_gazebo_ros_skid_steer_drive src/gazebo_ros_skid_steer_drive.cpp)
target_link_libraries(luna_gazebo_ros_skid_steer_drive ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
  luna_gazebo_ros_skid_steer_drive
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)
endif()
