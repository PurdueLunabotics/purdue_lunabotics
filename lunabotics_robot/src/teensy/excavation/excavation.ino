// include ROS and all messages
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Byte.h>

#include <lunabotics_arduino/drivetrain.h>
#include <lunabotics_arduino/exdep.h>

// create node handle
ros::NodeHandle nh;

void setup() {
  nh.initNode();
	Drivetrain drivetrain(&nh);
	ExDep exdep(&nh);
}

void loop() {
  // spin up the node once in the loop
  nh.spinOnce();
  delay(20);
}
