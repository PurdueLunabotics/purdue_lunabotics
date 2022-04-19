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
	static Stepper lead_screw_stepper(actuation_cfg.lead_screw.steps,
							   actuation_cfg.lead_screw.DIR1_P,
							   actuation_cfg.lead_screw.DIR2_P);

	// ACTUATION FUNCTIONS
	void init();
	void run_actuation(const lunabot_msgs::Actuation &actuation, ros::NodeHandle nh);
}

#endif
