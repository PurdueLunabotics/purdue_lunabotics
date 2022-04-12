#include "actuation.h"

Stepper stepper(STEPS, 0, 1, 2, 3); //TODO find ids
namespace actuation { 
	void init() {		

		// set all pwm and direction pins to output
		for(int i = 0; i < MOTOR_CNT; i++) {
			pinMode(drive_pins_[i], OUTPUT);
		}

		for(int i = 0; i < MOTOR_CNT; i++) { 
			pinMode(direction_pins_[i], OUTPUT);
		}
	}

	void _move_angle(int speed) {
		// (HIGH (CW and moves DOWN), LOW (CCW and moves UP))
		int direction = (speed > 0) ? LOW : HIGH;
		digitalWrite(direction_pins_[0], direction);
		digitalWrite(direction_pins_[1], direction);

		analogWrite(drive_pins_[0], abs(speed));
		analogWrite(drive_pins_[1], abs(speed));
	}

	void resetPID() {
		integral_accumulation = 0;
		prev_error = 0;
		prev_position = 0;
		prev_lin_speed = 0;
	}

	void _move_lead_screw(int speed) {
		int curr_pos = (long)analogRead(0) * STEPS / 1024.0; //Unsure if this works
		float curr_speed = (curr_pos - prev_position) / dt;
		float error = speed - curr_speed;
		float target = speed * F;
		target += error * P;
		integral_accumulation += error * dt;
		target += integral_accumulation * I;
		differential = (error - prev_error) / dt;
		target += differential * D;

		if(target > MAX_LIN_VEL) {
			target = MAX_LIN_VEL;
		} else if (target < -MAX_LIN_VEL) {
			target = -MAX_LIN_VEL;
		}

		stepper.setSpeed(target);

		prev_position = curr_pos;
		prev_lin_speed = curr_speed;
		prev_error = error;
	}

	// Negative value - move DOWN, 0 - STOP, positive value - move UP, range - [-1,1]
	void run_actuation(const lunabot_msgs::Actuation& actuation, ros::NodeHandle nh) {
		int lead_screw_speed = map(actuation.lead_screw_actuation, -MAX_SPEED, MAX_SPEED, -255, 255); // Range from [-255,255]
		int angle_speed = map(actuation.angle_actuation, -MAX_SPEED, MAX_SPEED, -255, 255); // Range from [-255,255]

		nh.logerror("Actuation:");  
		nh.logerror("  angle:");  
		nh.logerror(String(angle_speed).c_str());
		nh.logerror("  lead screw:");  
		nh.logerror(String(lead_screw_speed).c_str());

		_move_angle(lead_screw_speed);
		_move_lead_screw(angle_speed);
	}
}
