// include ROS and all messages
#include "ros.h"
#include <lunabot_msgs/RobotEffort.h>
#include <robot.hpp>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

void effort_cb(const lunabot_msgs::RobotEffort &effort) {
  actuation::cb(effort.lead_screw, effort.lin_act);
  drivetrain::cb(effort.left_drive, effort.right_drive);
  deposition::cb(effort.deposit);
  excavation::cb(effort.excavate);
}

ros::Subscriber<lunabot_msgs::RobotEffort> effort_sub("/effort", effort_cb);

std_msgs::Int16 lead_screw_curr_msg;
std_msgs::Int16 act_right_curr_msg;
std_msgs::Int16 exc_curr_msg;
std_msgs::Int16 dep_curr_msg;
std_msgs::Int16 drive_left_curr_msg;
std_msgs::Int16 drive_right_curr_msg;

ros::Publisher lead_screw_curr_pub("/current/act_lead_screw",
                                   &lead_screw_curr_msg);
ros::Publisher act_right_curr_pub("/current/act_right", &act_right_curr_msg);
ros::Publisher exc_curr_pub("/current/excavation", &exc_curr_msg);
ros::Publisher dep_curr_pub("/current/deposition", &dep_curr_msg);
ros::Publisher drive_left_curr_pub("/current/drive_left", &drive_left_curr_msg);
ros::Publisher drive_right_curr_pub("/current/drive_right",
                                    &drive_right_curr_msg);

void publish_state() {
  actuation::update(&(act_right_curr_msg.data), &(lead_screw_curr_msg.data));
  drivetrain::update(&(drive_left_curr_msg.data), &(drive_right_curr_msg.data));
  excavation::update(&(exc_curr_msg.data));
  deposition::update(&(dep_curr_msg.data));

  act_right_curr_pub.publish(&act_right_curr_msg);
  drive_left_curr_pub.publish(&drive_left_curr_msg);
  drive_right_curr_pub.publish(&drive_right_curr_msg);
  exc_curr_pub.publish(&exc_curr_msg);
  dep_curr_pub.publish(&dep_curr_msg);
}

// ros::Publisher effort_pub("/encoder/actuation");
// ros::Publisher effort_pub("/encoder/drive_left");
// ros::Publisher effort_pub("/encoder/drive_right");
// ros::Publisher effort_pub("/encoder/lin_act_left");
// ros::Publisher effort_pub("/encoder/lin_act_right");

void setup() {
  STMotorInterface::init_serial(ST_SERIAL, ST_BAUD_RATE);
  CurrentSensor::init_ads1115(&adc0);
  CurrentSensor::init_ads1115(&adc1);
  nh.initNode();
  nh.subscribe(effort_sub);
  nh.advertise(lead_screw_curr_pub);
  nh.advertise(act_right_curr_pub);
  nh.advertise(drive_left_curr_pub);
  nh.advertise(drive_right_curr_pub);
  nh.advertise(exc_curr_pub);
  nh.advertise(dep_curr_pub);
}

void loop() {
    nh.spinOnce();
    actuation::loop_once();
    drivetrain::loop_once();
    deposition::loop_once();
    excavation::loop_once();
    publish_state();
    delay(50);
}
