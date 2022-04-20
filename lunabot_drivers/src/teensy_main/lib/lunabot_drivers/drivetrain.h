#include <ros.h>
#include <lunabot_msgs/Drivetrain.h>
#include "config.h"

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

// enums for determining motor pin index
namespace drivetrain {

	// DRIVETRAIN FUNCTIONS
  	void init();
	void run_drivetrain(const lunabot_msgs::Drivetrain& drive_msg, ros::NodeHandle& nh); 
}

#endif
