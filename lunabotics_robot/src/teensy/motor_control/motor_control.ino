// include ROS and all messages
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Byte.h>

#include "drivetrain.h"
#include "exdep.h"

// create node handle
ros::NodeHandle node_handle;

// create subscriber nodes
ros::Subscriber<geometry_msgs::Twist> comm_subscriber("motor_command", drivetrain::driveRobot);
ros::Subscriber<std_msgs::Byte> exdep_subscriber("ex_dep_control", exdep::actuateExdep);

void setup() {
  // set all pwm and direction pins to output
  drivetrain::init();

  exdep::init();

  // initialize the node and add subscriber
  node_handle.initNode();
  node_handle.subscribe(comm_subscriber);
  node_handle.subscribe(exdep_subscriber);
}

void loop() {
  // spin up the node once in the loop
  node_handle.spinOnce();
  delay(20);
}
