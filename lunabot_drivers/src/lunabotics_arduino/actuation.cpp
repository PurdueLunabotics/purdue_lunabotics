#include "actuation.h"

namespace actuation { 
	void init() {
		// set all pwm and direction pins to output
		for(int i = 0; i < MOTOR_CNT; i++) {
			pinMode(drive_pins_[i], OUTPUT);
		}

		for(int i = 0; i < MOTOR_CNT; i++) { pinMode(direction_pins_[i], OUTPUT);
		}
	}

	void _move_angle(int speed) {
		// (HIGH (CW and moves DOWN), LOW (CCW and moves UP))
		int direction = (speed > 0) ? LOW : HIGH;
		digitalWrite(direction_pins_[1], direction);
		digitalWrite(direction_pins_[2], direction);

		analogWrite(drive_pins_[1], abs(speed));
		analogWrite(drive_pins_[2], abs(speed));
	}

	void _move_lead_screw(int speed) {
		int direction = (speed > 0) ? LOW : HIGH;
		digitalWrite(direction_pins_[0], direction);

		analogWrite(drive_pins_[0], abs(speed));
	}

	// Negative value - move DOWN, 0 - STOP, positive value - move UP, range - [-1,1]
	void run_actuation(const lunabotics_robot::Actuation& actuation, ros::NodeHandle nh) {
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
