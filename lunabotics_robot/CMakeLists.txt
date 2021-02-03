cmake_minimum_required(VERSION 3.0.2)
project(lunabotics_robot)

find_package(catkin REQUIRED COMPONENTS
 geometry_msgs
 std_msgs
 roscpp
 tf2_ros
 laser_geometry
)

catkin_package(
   #INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS std_msgs roscpp rospy geometry_msgs tf2_ros laser_geometry
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sensors_tf_broadcaster src/tf/sensors_tf_broadcaster.cpp)
target_link_libraries(sensors_tf_broadcaster ${catkin_LIBRARIES})

add_executable(joy_to_motor_control src/controller/manual_control_node.cpp)
target_link_libraries(joy_to_motor_control ${catkin_LIBRARIES})

add_executable(laser_to_point_cloud src/slam/laser_to_point_cloud.cpp)
target_link_libraries(laser_to_point_cloud ${catkin_LIBRARIES})
