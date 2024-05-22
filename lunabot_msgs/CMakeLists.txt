cmake_minimum_required(VERSION 3.5)
project(lunabot_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED COMPONENTS
# message_generation
# std_msgs
#)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)


find_package(rosidl_default_generators REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
    msg/RobotEffort.msg
    msg/RobotSensors.msg
    msg/RobotErrors.msg
    msg/Behavior.msg
    DEPENDENCIES std_msgs
)

# find_package(Boost REQUIRED COMPONENTS system filesystem thread)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lunabot_msgs
#   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
#)

ament_export_dependencies(message_runtime)
ament_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${Boost_INCLUDE_DIRS})
#include_directories(
  # include
#  ${catkin_INCLUDE_DIRS}
#)
