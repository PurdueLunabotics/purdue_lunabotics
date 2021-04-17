#include "deposition.h"

namespace deposition { 
	void init() {
		// set all pwm and direction pins to output
			pinMode(drive_pin_, OUTPUT);
			pinMode(direction_pin_, OUTPUT);
	}

	// logic for running a motor
	void _move_motor(int speed) {
		// (HIGH (CW and moves DOWN), LOW (CCW and moves UP))
		int direction = (speed > 0) ? LOW : HIGH;
		digitalWrite(direction_pin_, direction);

		analogWrite(drive_pin_, abs(speed));
	}

	// Negative value - move DOWN, 0 - STOP, positive value - move UP, range - [-1,1]
	void run_deposition(const std_msgs::Float64& speed, ros::NodeHandle nh) {
		int bin_move_speed = map(speed.data, -MAX_SPEED, MAX_SPEED, -255, 255); // Range from [-255,255]

		nh.logerror("Deposition:");  
		nh.logerror(String(bin_move_speed).c_str());

		//_move_motor(bin_move_speed);
	}
}
