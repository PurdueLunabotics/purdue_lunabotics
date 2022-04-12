#include <ros.h>
#include <lunabot_msgs/Actuation.h>
#include <Stepper.h>

#ifndef ACTUATION_H
#define ACTUATION_H


// enums for determining motor pin index
namespace actuation {
	// ex-dep motor counts
	const uint8_t MOTOR_CNT = 2;

	const int STEPS = 200; //Number of steps in motor;

	const int MAX_LIN_VEL = 0; //TODO find

	const float P = 0; //TODO find;
	const float I = 0; //TODO find
	const float D = 0; //TODO find
	const float F = 0; //TODO find, should be 1/max speed

	const float dt = 0.02; //TODO Find frequency, dt = 1/f

	int prev_position = 0;
	float prev_lin_speed = 0;
	float prev_error = 0;
	float integral_accumulation = 0;

	// ex-dep motor indices
	// corresponding entries are for the same motor
	const int drive_pins_[MOTOR_CNT] = {14, 18 };     //order: lead screw actuation, rotation actuation right, rotation actuation left 
	const int direction_pins_[MOTOR_CNT] = {15, 19 };
	const uint8_t MAX_SPEED = 1;	

	// ACTUATION FUNCTIONS
  void init();
	void _move_angle(int speed);
	void _move_lead_screw(int speed);
	void run_actuation(const lunabot_msgs::Actuation& actuation, ros::NodeHandle nh);  
}

#endif
