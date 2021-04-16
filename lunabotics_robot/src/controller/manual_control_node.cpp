#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

ros::Publisher motor_cmd_pub; 
geometry_msgs::Twist motor_cmd;

float lin_vel; // Forward backward linear velocity
float head_ang; // Heading angle

void joyCallback(const sensor_msgs::Joy& joy)
{
	motor_cmd = geometry_msgs::Twist();
	motor_cmd.linear.x = joy.axes[1];
	motor_cmd.angular.z = joy.axes[3];

	motor_cmd_pub.publish(motor_cmd);	
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_to_motor_control");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("joy", 1000, joyCallback);
  motor_cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ros::spin();

  return 0;
}
