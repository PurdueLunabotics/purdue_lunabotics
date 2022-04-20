#include "drivetrain.h"

namespace drivetrain { 
	void init() {
		// set all pwm and direction pins to output
		init_motor(drivetrain_cfg.left_front);
		init_motor(drivetrain_cfg.right_front);
		init_motor(drivetrain_cfg.left_back);
		init_motor(drivetrain_cfg.right_back);
	}

	void run_drivetrain(const lunabot_msgs::Drivetrain& drive_msg, ros::NodeHandle& nh) {
		/*
			Tank drive steering
		*/
		int8_t left_wheel = drive_msg.left; // left wheel vel 
		int8_t right_wheel = drive_msg.right; // right wheel vel 

		// calculate left and right chassis velocities
		int vel_l = map(left_wheel, -128, 127, -255, 255); // Range from [-255,255]
		int vel_r = map(right_wheel, -128, 127, -255, 255); // Range from [-255,255]
		vel_l = constrain(vel_l, -255, 255);
		vel_r = constrain(vel_r, -255, 255);
		//MotorDir left_vel_dir = (vel_l > 0) ? CCW : CW; 
		MotorDir left_vel_dir = (vel_l > 0) ? CW : CCW; // wiring issue
		MotorDir right_vel_dir = (vel_r > 0) ? CW : CCW; 

		nh.logerror("left_vel:");	
		nh.logerror(String(vel_l).c_str());
		nh.logerror("right_vel:");	
		nh.logerror(String(vel_r).c_str());

		write_motor(drivetrain_cfg.left_front,abs(vel_l),left_vel_dir);
		write_motor(drivetrain_cfg.left_back,abs(vel_l),left_vel_dir);
		write_motor(drivetrain_cfg.right_front,abs(vel_r),right_vel_dir);
		write_motor(drivetrain_cfg.right_back,abs(vel_r),right_vel_dir);
	}
}