#include <ros.h>
#include <std_msgs/Float64.h>

#ifndef DEPOSITION_H
#define DEPOSITION_H

// enums for determining motor pin index
namespace deposition {
	const uint8_t MAX_SPEED = 1;	

	// deposition motor indices
	const uint8_t drive_pin_ = 8; 
	const uint8_t direction_pin_ = 9;

	// DEPOSITION FUNCTIONS
  void init();
	void _move_motor(int command);
	void run_deposition(const std_msgs::Float64& command, ros::NodeHandle nh);  
}

#endif
