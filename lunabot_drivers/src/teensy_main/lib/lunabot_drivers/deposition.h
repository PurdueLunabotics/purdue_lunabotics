#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>

#include "actuator_config.h"
#include "sensor_config.h"

#ifndef DEPOSITION_H
#define DEPOSITION_H

// enums for determining motor pin index
namespace deposition
{

	extern State dep_curr;
	extern State dep_setp;
	// DEPOSITION FUNCTIONS
	void init();
	void run_deposition(const std_msgs::Int8 &command, ros::NodeHandle *nh);
	void move_to_setp();
}

#endif
