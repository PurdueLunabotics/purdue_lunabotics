#include "excavation.h"

namespace excavation {
	void init() {
		// stop 
		excavation_cfg.exc.st->stop();
	}

	void run_excavation(const std_msgs::Float64& speed, ros::NodeHandle nh) {
		int8_t excavate_speed = map(speed.data, -1, 1, -127, 127); // Range from [-255,255]
		write_serial_motor(excavation_cfg.exc, excavate_speed);
	}
}
