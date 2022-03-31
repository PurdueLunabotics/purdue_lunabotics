#include "drivetrain.h"

namespace drivetrain { 
	void init() {
		// set all pwm and direction pins to output
		for(int i = 0; i < MOTOR_CNT; i++) {
			pinMode(drive_pins_[i], OUTPUT);
		}

		for(int i = 0; i < MOTOR_CNT; i++) {
			pinMode(direction_pins_[i], OUTPUT);
			encoders_[i] = new Encoder(encoder_ids_[2 * i], encoder_ids_[2 * i + 1]);
		}

	}

	void _move_motor(motor_dir_t motor, int vel) {
		vel *= (motor % 2 == 0) ? -1 : 1; // Forward rotation of left motor is opposite to right 
		
		if(vel > 0) {
			digitalWrite(direction_pins_[motor], (direction_pins_inversions_[motor] ? LOW : HIGH)); // CW - HIGH, CCW - LOW 
		} else {
			digitalWrite(direction_pins_[motor], (direction_pins_inversions_[motor] ? HIGH : LOW));
		}

		// write the velocity to the pwm pin
		analogWrite(drive_pins_[motor], abs(vel));
	}

	void run_drivetrain(const lunabot_msgs::Drivetrain& drive_msg, ros::NodeHandle& nh) {
		/*
			Tank drive steering
		*/

		const float robot_width = 0; //TODO find

		float linear_vel = drive_msg.linear; // left wheel vel 
		float angular_vel = drive_msg.angular; // right wheel vel 

		float left_vel = (2 * linear_vel - robot_width * angular_vel) / 2.0
		float right_vel = (2 * linear_vel + robot_width * angular_vel) / 2.0 //TODO find units

		for(int i = 0; i < MOTOR_CNT; ++i) {
			float curr_pos = encoders_[i].read();
			float curr_velocity = (curr_pos - prev_positions_[i]) / delta_time;
			float goal_vel;
			if (i % 2 == 0) {
				goal_vel = left_vel;
			} else {
				goal_vel = right_vel;
			}
			float calculated_vel = goal_vel * pidf_values_[3]; //F
			float error = goal_vel - curr_velocity;
			calculated_vel += pidf_values_[0] * error; //P
			i_accumulations_[i] += error * delta_time;
			calculated_vel += pidf_values_[1] * i_accumulations_[i]; //I
			d_error = (error - prev_error_[i]) / delta_time;
			calculated_vel += pidf_values_[2] * d_error; //D

			if(calculated_vel < -1) { //Clamping speed
				calculated_vel = -1;
			} else if(calculated_vel > 1) {
				calculated_vel = 1;
			}

			calculated_vel *= 255 // mapping to arduino

			//Update motor
			_move_motor((motor_dir_t) motor, (int)(calculated_vel));

			//Updating previous values

			prev_positions_[i] = curr_pos;
			prev_velocities_[i] = curr_velocity;
			prev_error_[i] = error;
		}
	}
}
