#include <ros.h>
#include <std_msgs/Float32.h>
#include "actuator_config.h"
#include "sensor_config.h"
#include "HX711.h"
#include <IntervalTimer.h>

#ifndef EXCAVATION_H
#define EXCAVATION_H

namespace excavation
{
	// EXCAVATION FUNCTIONS
	void init();
	void run_excavation(const std_msgs::Float32 &speed, ros::NodeHandle *nh);
}

#endif
