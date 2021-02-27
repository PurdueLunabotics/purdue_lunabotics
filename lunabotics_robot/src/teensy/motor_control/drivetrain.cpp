#include <ros.h>
#include <geometry_msgs/Twist.h>

#include "drivetrain.h"

namespace drivetrain {

// chassis motor counts
const int NUM_CHASSIS_MOTOR = 4;
const int NUM_CHASSIS_DIRECTION = 4;

// chassis motor indices
// corresponding entries are for the same motor
const int driveMotorPins[NUM_CHASSIS_MOTOR] = {4, 8, 2, 6};           //order: front left, front right, back left, back right
const int driveDirectionPins[NUM_CHASSIS_DIRECTION] = {5, 9, 3, 7};

void init() {
  for(int i = 0; i < NUM_CHASSIS_MOTOR; i++) {
    pinMode(driveMotorPins[i], OUTPUT);
  }

  for(int i = 0; i < NUM_CHASSIS_DIRECTION; i++) {
    pinMode(driveDirectionPins[i], OUTPUT);
  }  
}

// enums for determining chassis motor pin index
#define FRONT 0
#define BACK 2
#define LEFT 0
#define RIGHT 1

// logic for running a motor
void moveChassisMotor(int front_or_back, int left_or_right, int vel) {
  // index is found from adding the front/back and left/right enums
  int ind = front_or_back + left_or_right;

  // if velocity is negative, reverse the motor
  if(vel > 0) {
    digitalWrite(driveDirectionPins[ind], HIGH);
  } else {
    digitalWrite(driveDirectionPins[ind], LOW);
  }

  // write the velocity to the pwm pin
  analogWrite(driveMotorPins[ind], abs(vel));
}

void driveRobot(const geometry_msgs::Twist& command) {
/*

    Skid-steering configuration is implemented.
 
    Uses the twist to define the velocity components of the robot. the lin component is the heading forward and backward velocity
    and the angular z (ang) is the heading angle.
    Both lin and ang have range of [-1,1]. 
  */

  double lin = command.linear.x; // heading velocity
  double ang = command.angular.z; // heading angle

  // calculate left and right chassis velocities
  int vel_l = constrain(lin * 128 - ang * 128, -255, 255); // Range from [-255,255]
  int vel_r = constrain(lin * 128 + ang * 128, -255, 255); // Range from [-255,255]

  // move each motor to the specified velocity
  moveChassisMotor(FRONT, LEFT, vel_l);
  moveChassisMotor(BACK, LEFT, vel_l);
  moveChassisMotor(FRONT, RIGHT, vel_r);
  moveChassisMotor(BACK, RIGHT, vel_r);  
}

}
