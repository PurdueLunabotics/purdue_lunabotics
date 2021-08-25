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

	void run_drivetrain(const geometry_msgs::Twist& cmd_vel, ros::NodeHandle& nh) {
		/*

			Skid-steering configuration is implemented.
	 
			Uses the twist to define the velocity components of the robot. the lin component is the heading forward and backward velocity
			and the angular z (ang) is the heading angle.
			Both lin and ang have range of [-1,1]. 
		*/

		double lin_v = cmd_vel.linear.x; // heading velocity
		double ang_v = cmd_vel.angular.z; // heading angle

		// calculate left and right chassis velocities
		int vel_l = map(lin_v - ang_v, -MAX_SPEED, MAX_SPEED, -255, 255); // Range from [-255,255]
		int vel_r = map(lin_v + ang_v, -MAX_SPEED, MAX_SPEED, -255, 255); // Range from [-255,255]
		vel_l = constrain(vel_l, -255, 255);
		vel_r = constrain(vel_r, -255, 255);

		nh.logerror("left_vel:");	
		nh.logerror(String(vel_l).c_str());
		nh.logerror("right_vel:");	
		nh.logerror(String(vel_r).c_str());

		// Order: FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
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
