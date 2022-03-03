#include <ros.h>
#include <std_msgs/Float64.h>

#ifndef EXCAVATION_H 
#define EXCAVATION_H 

namespace excavation {
		// EXCAVATION FUNCTIONS
		void init();
		void _move_motor(unsigned int speed);
		void run_excavation(const std_msgs::Float64& speed, ros::NodeHandle nh);  
		
		const uint8_t FIXED_DIRECTION = LOW; // Excavation system only moves in one direction
		const uint8_t MAX_SPEED = 1;

		// excavation motor indices
		const int drive_pin_ = 2; 
		const int direction_pin_ = 3;
}

#endif
