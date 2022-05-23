#include <ros.h>
#include <lunabot_msgs/Actuation.h>
#include <Arduino.h>
#include <Stepper.h>
#include "actuator_config.h"
#include "sensor_config.h"

#ifndef ACTUATION_H
#define ACTUATION_H

// enums for determining motor pin index
namespace actuation
{
	extern State lin_act_curr, lead_screw_curr;
	extern State lin_act_setp, lead_screw_setp;
	// ACTUATION FUNCTIONS
	void init();
	void stepper_step();
	void run_actuation(const lunabot_msgs::Actuation &actuation, ros::NodeHandle *nh);

	void move_to_setp(void);
}

#endif
