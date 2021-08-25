#include <ros.h>
#include <geometry_msgs/Twist.h>

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

// enums for determining motor pin index
namespace drivetrain {
	// chassis motor counts
	const uint8_t MOTOR_CNT = 4;
	const uint8_t MAX_SPEED = 1;
	typedef enum { FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT } motor_dir_t;

	// chassis motor indices
	// corresponding entries are for the same motors
	const uint8_t drive_pins_[MOTOR_CNT] = {0, 4, 2, 6};    //order: front left, front right, back left, back right
	const uint8_t direction_pins_[MOTOR_CNT] = {1, 5, 3, 7};

	// DRIVETRAIN FUNCTIONS
  void init();
	void run_drivetrain(const geometry_msgs::Twist& cmd_vel, ros::NodeHandle& nh); 
	void _move_motor(motor_dir_t motor, int vel);
}

#endif
