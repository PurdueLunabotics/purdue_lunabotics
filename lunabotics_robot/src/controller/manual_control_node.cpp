#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Byte.h"
#include <cstddef>

ros::Publisher motor_cmd_pub;
ros::Publisher ex_dep_control_pub;
geometry_msgs::Twist motor_cmd;
std_msgs::Byte ex_dep_cmd;

float lin_vel;  // Forward backward linear velocity
float head_ang; // Heading angle

void joyCallback(const sensor_msgs::Joy &joy) {
  motor_cmd = geometry_msgs::Twist();
  motor_cmd.linear.x = joy.axes[3];
  motor_cmd.angular.z = joy.axes[1];

  motor_cmd_pub.publish(motor_cmd);

  std::byte ex_dep_command{0};

  if(joy.buttons[0]) {
    ex_dep_command |= std::byte{1};
  } else if(joy.buttons[1]) {
    ex_dep_command |= std::byte{2};
  }

  ex_dep_command <<= 2;

  if(joy.buttons[2]) {
    ex_dep_command |= std::byte{1};
  } else if(joy.buttons[3]) {
    ex_dep_command |= std::byte{2};
  }

  ex_dep_command <<= 2;

  if(joy.buttons[4]) {
    ex_dep_command |= std::byte{1};
  } else if(joy.buttons[5]) {
    ex_dep_command |= std::byte{2};
  }

  ex_dep_cmd.data = ex_dep_command;
  ex_dep_control_pub.publish(ex_dep_cmd);

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "joy_to_motor_control");

  ros::NodeHandle n;

  ros::Subscriber drive_sub = n.subscribe("joy", 1000, joyCallback);
  motor_cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  motor_cmd_pub = n.advertise<std_msgs::Byte>("ex_dep_control", 1000);

  ros::spin();

  return 0;
}
