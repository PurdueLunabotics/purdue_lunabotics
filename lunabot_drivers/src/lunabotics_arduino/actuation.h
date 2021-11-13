#include <ros.h>
#include <lunabot_msgs/Actuation.h>

#ifndef ACTUATION_H
#define ACTUATION_H

// enums for determining motor pin index
namespace actuation {
	// ex-dep motor counts
	const uint8_t MOTOR_CNT = 3;

	// ex-dep motor indices
	// corresponding entries are for the same motor
	const int drive_pins_[MOTOR_CNT] = { 10, 14, 18 };     //order: lead screw actuation, rotation actuation right, rotation actuation left 
	const int direction_pins_[MOTOR_CNT] = { 11, 15, 19 };
	const uint8_t MAX_SPEED = 1;	

	// ACTUATION FUNCTIONS
  void init();
	void _move_angle(int speed);
	void _move_lead_screw(int speed);
	void run_actuation(const lunabot_msgs::Actuation& actuation, ros::NodeHandle nh);  
}

#endif
