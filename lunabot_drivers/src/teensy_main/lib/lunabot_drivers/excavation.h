#include <ros.h>
#include <std_msgs/Float64.h>
#include "actuator_config.h"
#include "sensor_config.h"

#ifndef EXCAVATION_H
#define EXCAVATION_H

namespace excavation
{
	// EXCAVATION FUNCTIONS
	void init();
	void run_excavation(const std_msgs::Float64 &speed, ros::NodeHandle *nh);
}

#endif
