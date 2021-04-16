#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <cstddef>

ros::Publisher drivetrain_pub;
ros::Publisher deposition_pub;
ros::Publisher actuation_pub;

float lin_vel;  // Forward backward linear velocity
float head_ang; // Heading angle

void joyCallback(const sensor_msgs::Joy& joy)
{
	geometry_msgs::Twist cmd_vel;
	cmd_vel = geometry_msgs::Twist();
	cmd_vel.linear.x = joy.axes[1];
	cmd_vel.angular.z = joy.axes[3];
	drivetrain_pub.publish(cmd_vel);
	/*

	std_msgs::Float64 deposition_msg;

	deposition_msg.data = 5;
	deposition_pub.publish(deposition_msg);

	std_msgs::Float64 actuation_msg;

	actuation_msg.data = joy.axes[1];

	actuation_pub.publish(actuation_msg);
	*/
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "joy_to_motor_control");

  ros::NodeHandle n;

  ros::Subscriber drive_sub = n.subscribe("joy", 1000, joyCallback);
  drivetrain_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  deposition_pub = n.advertise<std_msgs::Float64>("deposition", 1000);
  actuation_pub = n.advertise<std_msgs::Float64>("actuation", 1000);

  ros::spin();

  return 0;
}
