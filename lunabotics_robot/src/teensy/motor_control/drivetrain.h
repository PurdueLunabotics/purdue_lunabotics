#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <ros.h>
#include <geometry_msgs/Twist.h>

namespace drivetrain {
  void init();
  void moveChassisMotor(int front_or_back, int left_or_right, int vel);
  void driveRobot(const geometry_msgs::Twist& command);  
}

#endif
