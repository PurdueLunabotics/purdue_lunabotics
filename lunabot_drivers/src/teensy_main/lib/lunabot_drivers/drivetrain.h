#include <ros.h>
#include <lunabot_msgs/Drivetrain.h>
#include <Encoder.h>
#include <IntervalTimer.h>
#include "actuator_config.h"
#include "sensor_config.h"

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#define P_CTRL 0.2
#define D_CTRL 0.02
#define DT_MILLIS 600000U
#define DT 0.6
#define F_ 435U

// enums for determining motor pin index
namespace drivetrain
{

	struct WheelControl
	{
		MotorConfig motor;
		float prev_err;
		int prev_pos;
		float curr_vel;
		float setp;		// wheel vel [1,-1]
		MotorDir forward;	// spins motor torwards excavation tool (front)
		MotorDir backward; // spins motor towards deposition (back)
	}; 

	extern WheelControl front_left_ctrl, front_right_ctrl, back_left_ctrl, back_right_ctrl;

	// DRIVETRAIN FUNCTIONS
	void init();
	void run_drivetrain(const lunabot_msgs::Drivetrain &drive_msg, ros::NodeHandle *nh);
	void ctrl_loop(void);
}

#endif
