#include <ros.h>
#include <std_msgs/Int32.h>

#ifndef DEPOSITION_H 
#define DEPOSITION_H 

class Deposition {
	public:
  	Deposition(ros::NodeHandle& nodehandle);
	
	private:
		ros::NodeHandle nh_;
		ros::Subscriber<std_msgs::Int32, Deposition> deposition_sub_;

		void _move_motor(int depose);
		void _deposition_cb(const std_msgs::Int32& command);  

		const uint8_t FIXED_SPEED = 255 / 2;	

		// deposition motor indices
		const int drive_pin_ = 18; 
		const int direction_pin_ = 19;
};

#endif
