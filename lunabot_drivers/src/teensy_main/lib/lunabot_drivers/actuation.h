#include <ros.h>
#include <lunabot_msgs/Actuation.h>
#include <Arduino.h>
#include <Stepper.h>
#include "config.h"


#ifndef ACTUATION_H
#define ACTUATION_H

// enums for determining motor pin index
namespace actuation
{
	// ACTUATION FUNCTIONS
	void init();
	void stepper_step();
	void run_actuation(const lunabot_msgs::Actuation &actuation, ros::NodeHandle nh);
}

#endif
