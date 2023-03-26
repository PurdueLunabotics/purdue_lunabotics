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

/*
ros::Publisher effort_pub("/current/act_lead_screw");
ros::Publisher effort_pub("/current/act_left");
ros::Publisher effort_pub("/current/excavation");
ros::Publisher effort_pub("/current/deposition");
ros::Publisher effort_pub("/current/drive_left");
ros::Publisher effort_pub("/current/drive_right");

// ros::Publisher effort_pub("/encoder/actuation");
// ros::Publisher effort_pub("/encoder/drive_left");
// ros::Publisher effort_pub("/encoder/drive_right");
// ros::Publisher effort_pub("/encoder/lin_act_left");
*/

void publish() {}

void setup() {
    STMotorInterface::init_serial(ST_SERIAL, ST_BAUD_RATE);
    nh.initNode();
    nh.subscribe(effort_sub);
}

void loop() {
    nh.spinOnce();
    delay(10);
}
