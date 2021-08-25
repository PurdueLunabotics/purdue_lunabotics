cmake_minimum_required(VERSION 3.0.2)
project(lunabotics_gazebo_plugins)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS gazebo_ros)

## System dependencies are found with CMake's conventions
find_package(gazebo REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

catkin_package(
	INCLUDE_DIRS 
	LIBRARIES
	CATKIN_DEPENDS
	DEPENDS
		gazebo_ros
)
