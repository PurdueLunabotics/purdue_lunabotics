#include <ros.h>
#include <std_msgs/Byte.h>

#ifndef EXCAVATION_H 
#define EXCAVATION_H 

class Excavation {
	public:
  	Excavation(ros::NodeHandle* nodehandle);
	
	private:
		ros::NodeHandle* nh_;
		ros::Subscriber<std_msgs::Byte> excavation_sub_;

		void _move_motor();
		void _excavation_cb(const std_msgs::Byte& command);  
		
		const uint8_t FIXED_DIRECTION = LOW;	
		const uint8_t FIXED_SPEED = 255;	

		// excavation motor indices
		const int drive_pin_ = 18; 
		const int direction_pin_ = 19;
};

#endif
