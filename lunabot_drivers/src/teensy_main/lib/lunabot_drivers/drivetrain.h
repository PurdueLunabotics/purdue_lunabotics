#include <ros.h>
#include <lunabot_msgs/Drivetrain.h>
#include <Encoder.h>
#include "actuator_config.h"
#include "sensor_config.h"

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#define P_CTRL 0.2
#define D_CTRL 0.02
#define DT_MILLIS 600000U
#define DT 0.6
#define F 435U


// enums for determining motor pin index
namespace drivetrain {

	struct WheelControl {
		MotorConfig motor;
		EncoderConfig enc; 
		volatile float prev_err;
		volatile int prev_pos;
		volatile float setp;
	} front_left, front_right, back_left, back_right;

	// DRIVETRAIN FUNCTIONS
  	void init();
	void run_drivetrain(const lunabot_msgs::Drivetrain& drive_msg, ros::NodeHandle* nh); 
}

#endif
