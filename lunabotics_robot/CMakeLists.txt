cmake_minimum_required(VERSION 3.0.2)
project(lunabotics_robot)

find_package(catkin REQUIRED COMPONENTS
 geometry_msgs
 std_msgs
 roscpp
 robot_localization
 message_generation
 tf2_ros
 laser_geometry
 sensor_msgs 
)

include_directories(
	#include
  ${catkin_INCLUDE_DIRS}
)

## Generate messages in the 'msg' folder
add_message_files(FILES
  Actuation.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
   #INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS std_msgs roscpp rospy geometry_msgs message_runtime tf2_ros laser_geometry sensor_msgs 
)



add_executable(laser_to_point_cloud src/slam/laser_to_point_cloud.cpp)
target_link_libraries(laser_to_point_cloud ${catkin_LIBRARIES})
