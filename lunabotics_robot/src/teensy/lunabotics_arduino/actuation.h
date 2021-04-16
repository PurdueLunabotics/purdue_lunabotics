/*
#include <ros.h>
#include <std_msgs/Int32.h>

#ifndef ACTUATION_H
#define ACTUATION_H

// enums for determining exdep motor pin index

class Actuation {
	public:
  	Actuation(ros::NodeHandle* nodehandle);
	
	private:
		ros::NodeHandle nh_;
		ros::Subscriber<std_msgs::Int32> actuation_sub_;

		void _actuation_cb(const std_msgs::Int32& command);  
		void _move_angle(int angle);

		void _move_lead_screw(int actuate);

		// ex-dep motor counts
		static const uint8_t MOTOR_CNT_ = 3;
		static const uint8_t FIXED_SPEED_ = 255;

		// ex-dep motor indices
		// corresponding entries are for the same motor
		const int drive_pins_[MOTOR_CNT_] = { 10, 14, 18 };     //order: lead screw actuation, rotation actuation right, rotation actuation left 
		const int direction_pins_[MOTOR_CNT_] = { 11, 15, 19 };
};

#endif
*/
