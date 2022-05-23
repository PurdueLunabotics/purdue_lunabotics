#include "deposition.h"

namespace deposition
{
	State dep_curr = {.state = INT(DepState::STORED)};

	void dep_hall_cb()
	{
		if (digitalRead(DEP_HALL_PIN) == LOW)
		{
			dep_curr.state = (dep_curr.state + 1) % INT(DepState::CNT);
			stop_motor(deposition_cfg.dep_motor);
		}
	}

	void init()
	{
		// set all pwm and direction pins to output
		init_motor(deposition_cfg.dep_motor);
		init_hall(DEP_HALL_PIN, dep_hall_cb, &dep_curr);
	}

	void move_to_setp(void)
	{
		if (dep_curr.state != dep_setp.state && dep_setp.state != INT(DepState::STOPPED))
		{
			MotorDir dir = (dep_curr.state == INT(DepState::STORED)) ? CCW : CW; // CCW rotates deposition up when stored, CW retracts deposition down when extended
			write_motor(deposition_cfg.dep_motor,
						deposition_cfg.dep_motor.MAX_PWM, dir);
		}
	}

	void run_deposition(const std_msgs::Int8 &speed, ros::NodeHandle *nh)
	{
		MotorDir dep_dir = (speed.data > 0) ? CCW : CW; // CCW rotates deposition up, CW retracts deposition down
		bool dep_en = speed.data != 0;

		// nh.logerror("Deposition:");
		// nh.logerror(String(bin_move_speed).c_str());
		if (dep_en)
		{
			write_motor(deposition_cfg.dep_motor,
						deposition_cfg.dep_motor.MAX_PWM, dep_dir);
		}
		else
		{
			stop_motor(deposition_cfg.dep_motor);
		}
	}
}
