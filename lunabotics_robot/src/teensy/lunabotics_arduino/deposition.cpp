#include "deposition.h"

Deposition::Deposition(ros::NodeHandle& nh): nh_(nh), deposition_sub_("deposition", &Deposition::_deposition_cb, this) {
	nh_.subscribe(deposition_sub_);

  pinMode(drive_pin_, OUTPUT);

  pinMode(direction_pin_, OUTPUT);
}

// logic for running a motor
void Deposition::_move_motor(int depose) {
	// (HIGH (CW and moves DOWN), LOW (CCW and moves UP))
	int direction = (depose > 0) ? LOW : HIGH;
	digitalWrite(direction_pin_, direction);

  analogWrite(drive_pin_, FIXED_SPEED);
}

// Negative value - move DOWN, 0 - STOP, positive value - move UP 
void Deposition::_deposition_cb(const std_msgs::Int32& depose) {

  nh_.logerror("Deposition:");  
  nh_.logerror(String(depose.data).c_str());

  //_move_motor(depose.data);
}
