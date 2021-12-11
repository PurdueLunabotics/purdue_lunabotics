#include "drivetrain.h"

namespace drivetrain { 
	void init() {
		// set all pwm and direction pins to output
		for(int i = 0; i < MOTOR_CNT; i++) {
			pinMode(drive_pins_[i], OUTPUT);
		}

		for(int i = 0; i < MOTOR_CNT; i++) {
			pinMode(direction_pins_[i], OUTPUT);
		}
	}

	void _move_motor(motor_dir_t motor, int vel) {
		vel *= (motor % 2 == 0) ? -1 : 1; // Forward rotation of left motor is opposite to right 
		
		if(vel > 0) {
			digitalWrite(direction_pins_[motor], HIGH); // CW - HIGH, CCW - LOW 
		} else {
			digitalWrite(direction_pins_[motor], LOW);
		}

		// write the velocity to the pwm pin
		analogWrite(drive_pins_[motor], abs(vel));
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

		nh.logerror("left_vel:");	
		nh.logerror(String(vel_l).c_str());
		nh.logerror("right_vel:");	
		nh.logerror(String(vel_r).c_str());

		// Order: FRONT_LEFT 0, FRONT_RIGHT 1, BACK_LEFT 2, BACK_RIGHT 3
		// move each motor to the specified velocity
		for (int motor = FRONT_LEFT; motor <= BACK_RIGHT; motor++) {
			if(motor % 2 == 0) { // moves left motors
				_move_motor((motor_dir_t) motor, vel_l);
			}
			else {
				_move_motor((motor_dir_t) motor, vel_r); // moves right motors
			}
		}
	}
}
