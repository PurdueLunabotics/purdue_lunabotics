#include "deposition.h"

namespace deposition { 
	void init() {
		// set all pwm and direction pins to output
		init_motor(deposition_cfg.dep_motor);
	}

	void run_deposition(const std_msgs::Int8& speed, ros::NodeHandle nh) {
		MotorDir dep_dir = (speed.data > 0) ? CCW : CW; // CCW rotates deposition up, CW retracts deposition down 
		bool dep_en = speed.data != 0;

		// nh.logerror("Deposition:");  
		// nh.logerror(String(bin_move_speed).c_str());
		if(dep_en) {
			write_motor(deposition_cfg.dep_motor,
						deposition_cfg.dep_motor.MAX_PWM,dep_dir);
		}
		else {
			stop_motor(deposition_cfg.dep_motor);
		}
		
	}
}
