/*
#include <ros.h>
#include <geometry_msgs/Twist.h>

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

// enums for determining chassis motor pin index
#define FRONT 0
#define BACK 2
#define LEFT 0
#define RIGHT 1

class Drivetrain {
	public:
  	Drivetrain(ros::NodeHandle* nodehandle);
  	Drivetrain();
		ros::Subscriber<geometry_msgs::Twist, Drivetrain> drivetrain_sub_;
	
	private:
		ros::NodeHandle* nh_;


		void _move_motor(int front_or_back, int left_or_right, int vel);
		void _drivetrain_cb(const geometry_msgs::Twist& command);  

		// chassis motor counts
		static const uint8_t MOTOR_CNT_ = 4;
		const double MAX_VEL = 1;

		// chassis motor indices
		// corresponding entries are for the same motor
		const int drive_pins_[MOTOR_CNT_] = {0, 4, 2, 6};          //order: front left, front right, back left, back right
		const int direction_pins_[MOTOR_CNT_] = {1, 5, 3, 7};


};

#endif
*/
