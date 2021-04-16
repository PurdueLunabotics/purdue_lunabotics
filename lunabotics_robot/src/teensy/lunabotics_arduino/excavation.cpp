/*
#include "excavation.h"

Excavation::Excavation(ros::NodeHandle* nodehandle) {
	_nh = nodehandle;
 	excavation_sub_ = nh_.subscribe("excavation", 5, &Excavation::_excavation_cb, this);

  pinMode(drive_pin_, OUTPUT);

  pinMode(direction_pin_, OUTPUT);
}

// logic for running a motor
void Excavation::_move_motor() {

  digitalWrite(direction_pin_, FIXED_DIRECTION);
  analogWrite(drive_pin_, FIXED_SPEED);
}

void Excavation::_excavation_cb(const std_msgs::Byte& excavate) {

  nh_.logerror("Excavation:");  
  nh_.logerror(String(excavate).c_str());

  //_move_motor();
}
*/
