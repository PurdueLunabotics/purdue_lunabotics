#include "actuation.h"

namespace actuation
{

	static StepperDir lead_screw_dir;
	static uint8_t lead_screw_en;

	Stepper lead_screw_stepper(actuation_cfg.lead_screw.steps,
								actuation_cfg.lead_screw.DIR1_P,
								actuation_cfg.lead_screw.DIR2_P);

	void init()
	{
		// set all pwm and direction pins to output
		init_stepper(actuation_cfg.lead_screw,&lead_screw_stepper);
		stepper_off(actuation_cfg.lead_screw);
		lead_screw_en = 0;
	}

	void stepper_step() {
		if(lead_screw_en) {
			stepper_step(actuation_cfg.lead_screw,&lead_screw_stepper,lead_screw_dir);
		}
	}

	// Negative value - move DOWN, 0 - STOP, positive value - move UP, range - [-1,1]
	void run_actuation(const lunabot_msgs::Actuation &actuation, ros::NodeHandle nh)
	{
		lead_screw_dir = (actuation.lead_screw > 0) ? EXTEND : RETRACT; 
		lead_screw_en = actuation.lead_screw != 0;

		MotorDir angle_dir = (actuation.angle > 0) ? CW : CCW; 

		if(actuation.angle != 0) {
			write_serial_motor(actuation_cfg.lin_act, actuation_cfg.lin_act.MAX_PWM * actuation.angle);
		}
		else {
			write_serial_motor(actuation_cfg.lin_act, 0);
		}

		//nh.logerror("lead screw:");
		if(lead_screw_en) {
			//nh.logerror("ON");
			stepper_on(actuation_cfg.lead_screw);
		}
		else {
			//nh.logerror("STOP");
			stepper_off(actuation_cfg.lead_screw);
		}
	}
}
