#include "actuation.h"

// TODO: Implement custom messages 

Actuation::Actuation(ros::NodeHandle* nodehandle):nh_(*nodehandle) {
 	actuation_sub_ = nh_.subscribe("actuation", 5, &Actuation::_actuation_cb, this);

  for(int i = 0; i < MOTOR_CNT_; i++) {
    pinMode(drive_pins_[i], OUTPUT);
  }

  for(int i = 0; i < MOTOR_CNT_; i++) {
    pinMode(direction_pins_[i], OUTPUT);
  }  
}

// TODO: Implement move angle
void Actuation::_move_angle(int angle) {
	// (HIGH (CW and moves DOWN), LOW (CCW and moves UP))
	int direction = (depose > 0) ? LOW : HIGH;
	digitalWrite(direction_pins_[1], direction);
	digitalWrite(direction_pins_[2], direction);

  analogWrite(drive_pins_[1], FIXED_SPEED);
  analogWrite(drive_pins_[2], FIXED_SPEED);
}

void Actuation::_move_lead_screw(int actuate) {
	int direction = (actuate > 0) ? LOW : HIGH;
	digitalWrite(direction_pins_[0], direction);

  analogWrite(drive_pin_[0], FIXED_SPEED);
}

void Actuation::_actuation_cb(const std_msgs::Int32& actuate) {

  nh_.logerror("Actuation:");  
  nh_.logerror(String(depose.data).c_str());

  //_move_lead_screw(depose.data);
}
*/
