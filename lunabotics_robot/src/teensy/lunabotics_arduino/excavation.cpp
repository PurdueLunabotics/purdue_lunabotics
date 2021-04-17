#include "excavation.h"

namespace excavation {
	void init() {
		// set all pwm and direction pins to output
			pinMode(drive_pin_, OUTPUT);
			pinMode(direction_pin_, OUTPUT);
	}

	// logic for running the excavation motor
	void _move_motor(unsigned int speed) {

		digitalWrite(direction_pin_, FIXED_DIRECTION);
		analogWrite(drive_pin_, abs(speed));
	}

	void run_excavation(const std_msgs::Float64& speed, ros::NodeHandle nh) {
		unsigned int excavate_speed = abs(map(speed.data, -MAX_SPEED, MAX_SPEED, -255, 255)); // Range from [-255,255]

		nh.logerror("Excavation:");  
		nh.logerror(String(excavate_speed).c_str());

		//_move_motor(excavate_speed);
	}
}
