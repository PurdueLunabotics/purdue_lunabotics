cmake_minimum_required(VERSION 3.0.2)
project(lunabotics_robot)

find_package(catkin REQUIRED COMPONENTS
 geometry_msgs
 roscpp
 tf
)

catkin_package(
   #INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp rospy geometry_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sensors_tf_broadcaster src/tf/sensors_tf_broadcaster.cpp)
target_link_libraries(sensors_tf_broadcaster ${catkin_LIBRARIES})
