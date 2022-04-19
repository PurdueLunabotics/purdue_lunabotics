#include "excavation.h"

namespace excavation {
	void init() {
		// set all pwm and direction pins to output
		init_motor(excavation_cfg.exc);
	}

	void run_excavation(const std_msgs::Float64& speed, ros::NodeHandle nh) {
		unsigned int excavate_speed = abs(map(speed.data, -1, 1, -255, 255)); // Range from [-255,255]
		MotorDir excavate_dir = (speed.data > 0) ? CW : CCW; 

		nh.logerror("Excavation:");  
		nh.logerror(String(excavate_speed).c_str());

		write_motor(excavation_cfg.exc,excavate_speed, excavate_dir);
	}
}
