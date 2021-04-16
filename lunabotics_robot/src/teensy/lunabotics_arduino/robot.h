#include <ros.h>
#include <deposition.h>
#include <std_msgs/Int32.h>

#ifndef ROBOT_H 
#define ROBOT_H 

class Robot {
	public:
		ros::NodeHandle nh;
		Deposition deposition;
		ros::Subscriber<std_msgs::Int32, Deposition> deposition_sub;

		Robot()
		: deposition_sub("/deposition", &Deposition::deposition_cb, deposition)
		{  // Constructor
				nh.initNode()
				nh.subscribe(deposition_sub);
		}
};

#endif
