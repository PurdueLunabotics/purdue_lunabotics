#include "excavation.h"

namespace excavation {
	void init() {
		// set all pwm and direction pins to output
		stop_motor(excavation_cfg.exc);

		write_serial_motor(excavation_cfg.exc, 0);
	}

	void run_excavation(const std_msgs::Float64& speed, ros::NodeHandle nh) {
		unsigned int excavate_speed = abs(map(speed.data, -1, 1, -127, 127)); // Range from [-255,255]
		write_serial_motor(excavation_cfg.exc, excavate_speed);
	}
}
