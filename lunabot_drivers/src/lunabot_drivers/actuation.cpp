#include "actuation.h"

namespace actuation
{
	void init()
	{
		// set all pwm and direction pins to output
		init_motor(actuation_cfg.left_lin_act);
		init_motor(actuation_cfg.right_lin_act);
		init_stepper(actuation_cfg.lead_screw, &lead_screw_stepper);
	}

	// Negative value - move DOWN, 0 - STOP, positive value - move UP, range - [-1,1]
	void run_actuation(const lunabot_msgs::Actuation &actuation, ros::NodeHandle nh)
	{
		StepperDir lead_screw_dir = (actuation.lead_screw > 0) ? EXTEND : RETRACT; 
		bool lead_screw_en = actuation.lead_screw != 0;
		MotorDir angle_dir = (actuation.angle > 0) ? CW : CCW; 
		bool angle_en = actuation.angle != 0;

		nh.logerror("Actuation:");
		nh.logerror("  angle:");
		nh.logerror(String(angle_en).c_str());

		nh.logerror("  lead screw:");
		nh.logerror(String(lead_screw_en).c_str());

		if(angle_en) {
			write_motor(actuation_cfg.left_lin_act,
						actuation_cfg.left_lin_act.MAX_PWM,angle_dir);
			write_motor(actuation_cfg.right_lin_act,
						actuation_cfg.right_lin_act.MAX_PWM,angle_dir);
		}
		else {
			stop_motor(actuation_cfg.left_lin_act);
			stop_motor(actuation_cfg.right_lin_act);
		}

		if(lead_screw_en) {

			step(actuation_cfg.lead_screw, &lead_screw_stepper, lead_screw_dir);
		}
	}
}
