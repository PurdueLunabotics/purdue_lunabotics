/*
#include "drivetrain.h"

Drivetrain::Drivetrain() : drivetrain_sub_("cmd_vel", &Drivetrain::_drivetrain_cb, this) {
	nh_->subscribe(drivetrain_sub_);
}

Drivetrain::Drivetrain(ros::NodeHandle* nodehandle) {
	nh_ = nodehandle;

  for(int i = 0; i < MOTOR_CNT_; i++) {
    pinMode(drive_pins_[i], OUTPUT);
  }

  for(int i = 0; i < MOTOR_CNT_; i++) {
    pinMode(direction_pins_[i], OUTPUT);
  }  
}

// logic for running a motor
void Drivetrain::_move_motor(int front_or_back, int left_or_right, int vel) {
  // index is found from adding the front/back and left/right enums
  int ind = front_or_back + left_or_right;
  int forward = (!left_or_right) ? HIGH : LOW; // left motors are opposite
  // if velocity is negative, reverse the motor
  
  if(vel > 0) {
    digitalWrite(direction_pins_[ind], forward);
  } else {
    digitalWrite(direction_pins_[ind], !forward);
  }

  // write the velocity to the pwm pin
  analogWrite(drive_pins_[ind], abs(vel));
}

void Drivetrain::_drivetrain_cb(const geometry_msgs::Twist& cmd_vel) {

    Skid-steering configuration is implemented.
 
    Uses the twist to define the velocity components of the robot. the lin component is the heading forward and backward velocity
    and the angular z (ang) is the heading angle.
    Both lin and ang have range of [-1,1]. 

  double lin_v = cmd_vel.linear.x; // heading velocity
  double ang_v = cmd_vel.angular.z; // heading angle

  // calculate left and right chassis velocities
  int vel_l = map(lin_v - ang_v, -MAX_VEL, MAX_VEL, -255, 255); // Range from [-255,255]
  int vel_r = map(lin_v + ang_v, -MAX_VEL, MAX_VEL, -255, 255); // Range from [-255,255]
  vel_l = constrain(vel_l, -255, 255);
  vel_r = constrain(vel_r, -255, 255);

  nh_->logerror("left_vel:"); 
  nh_->logerror(String(vel_l).c_str());
  nh_->logerror("right_vel:");  
  nh_->logerror(String(vel_r).c_str());

  // move each motor to the specified velocity
 // _move_motor(FRONT, LEFT, vel_l);
 // _move_motor(BACK, LEFT, vel_l);
 // _move_motor(FRONT, RIGHT, vel_r);
 // _move_motor(BACK, RIGHT, vel_r);
}
*/
