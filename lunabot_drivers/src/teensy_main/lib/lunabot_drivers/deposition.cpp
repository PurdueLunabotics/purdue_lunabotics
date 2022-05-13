#include "deposition.h"

namespace deposition
{

	void dep_hall_cb()
	{

		if(digitalRead(DEP_HALL_PIN) == LOW) {
			dep_hall.dep_state = static_cast<DepState>((dep_hall.dep_state + 1) % DepState::CNT);
			stop_motor(deposition_cfg.dep_motor);
			dep_hall.lim = AT_LIMIT;
		}
		else {
			dep_hall.lim = FREE;

		}
	}

	void init()
	{
		// set all pwm and direction pins to output
		init_motor(deposition_cfg.dep_motor);
		dep_hall = { .dep_state = DepState::STORED, .lim = AT_LIMIT};
		init_hall(DEP_HALL_PIN, dep_hall_cb);
	}


	void run_deposition(const std_msgs::Int8 &speed, ros::NodeHandle *nh)
	{
		MotorDir dep_dir = (speed.data > 0) ? CCW : CW; // CCW rotates deposition up, CW retracts deposition down
		bool dep_en = speed.data != 0;

		if (dep_hall.lim == AT_LIMIT) {
			if(dep_hall.dep_state == DepState::STORED && dep_dir == CW) {
				return;
			}
			if(dep_hall.dep_state == DepState::FULL_EXT && dep_dir == CCW) {
				return;
			}
		}

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
