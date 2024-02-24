#include <lunabot_control/mpc.h>

bool traversal_enabled = true;

void traversal_bool_callback(const std_msgs::Bool& msg) {
  traversal_enabled = msg.data;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle nh;

  MPC mpc(&nh);

  double frequency;
  ros::param::get("~frequency", frequency);
  ros::Rate rate(frequency);

  nh.subscribe("/behavior/traversal_enabled", 10, traversal_bool_callback);

  while (ros::ok()) {
    ros::spinOnce();
    if (traversal_enabled) {
      mpc.calculate_velocity();
    }
    rate.sleep();
  }

}
