#include "drivetrain.h"

namespace drivetrain { 
	void init() {
		// set all pwm and direction pins to output
		init_motor(drivetrain_cfg.left);
		init_motor(drivetrain_cfg.right);
	}

	void run_drivetrain(const lunabot_msgs::Drivetrain& drive_msg, ros::NodeHandle& nh) {
		/*
			Tank drive steering
		*/
		int8_t left_wheel = std::min(drive_msg.left,127); // left wheel vel 
		int8_t right_wheel = std::min(drive_msg.right,127); // right wheel vel 

		// calculate left and right chassis velocities

		write_serial_motor(drivetrain_cfg.left,left_wheel);
		write_serial_motor(drivetrain_cfg.right,right_wheel);
	}
}