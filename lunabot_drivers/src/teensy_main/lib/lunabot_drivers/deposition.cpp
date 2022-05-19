#include "deposition.h"

namespace deposition
{
	HallSensor dep_hall = { .state = INT(DepState::STORED), .init_state=INT(DepState::STORED), .last_debounce_time = 0};

	void dep_hall_cb()
	{

		if (millis() - dep_hall.last_debounce_time > DEBOUNCE_DELAY_MILLIS) {
			dep_hall.state = (dep_hall.state + 1) % INT(DepState::CNT);
			stop_motor(deposition_cfg.dep_motor);
		}

		dep_hall.last_debounce_time = millis();
	}

	void init()
	{
		// set all pwm and direction pins to output
		init_motor(deposition_cfg.dep_motor);
		init_hall(DEP_HALL_PIN, dep_hall_cb, &dep_hall);
	}


	void run_deposition(const std_msgs::Int8 &speed, ros::NodeHandle *nh)
	{
		MotorDir dep_dir = (speed.data > 0) ? CCW : CW; // CCW rotates deposition up, CW retracts deposition down
		bool dep_en = speed.data != 0;


		// nh.logerror("Deposition:");
		// nh.logerror(String(bin_move_speed).c_str());
		if (dep_en)
		{
			if(dep_hall.state == INT(DepState::STORED) && dep_dir == CW) {
				stop_motor(deposition_cfg.dep_motor);
			}
			else if(dep_hall.state == INT(DepState::FULL_EXT) && dep_dir == CCW) {
				stop_motor(deposition_cfg.dep_motor);
			}
			else {
				write_motor(deposition_cfg.dep_motor,
							deposition_cfg.dep_motor.MAX_PWM, dep_dir);
			}
		}
		else
		{
			stop_motor(deposition_cfg.dep_motor);
		}
	}
}
