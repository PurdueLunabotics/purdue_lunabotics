// include ROS and all messages
#include "ros.h"
#include <lunabot_msgs/RobotEffort.h>
#include <robot.hpp>

ros::NodeHandle nh;

void effort_cb(const lunabot_msgs::RobotEffort &effort) {
  actuation::cb(effort.lead_screw, effort.lin_act);
  drivetrain::cb(effort.left_drive, effort.right_drive);
  deposition::cb(effort.deposit);
  excavation::cb(effort.excavate);
}

ros::Subscriber<lunabot_msgs::RobotEffort> effort_sub("/effort", effort_cb);

void setup() {
  STMotorInterface::init_serial(ST_SERIAL, ST_BAUD_RATE);
  nh.initNode();
  nh.subscribe(effort_sub);
}

void loop() {
  nh.spinOnce();
  actuation::run();
  delay(10);
}
