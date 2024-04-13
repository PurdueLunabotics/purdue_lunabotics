#include <lunabot_control/mpc.h>

bool traversal_enabled = true;

MPC* mpc_ptr = NULL;

void traversal_bool_callback(const std_msgs::Bool& msg) {
  traversal_enabled = msg.data;

  if (!traversal_enabled & mpc_ptr != NULL) {
    mpc_ptr->publish_velocity_(0, 0);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle nh;

  MPC mpc(&nh);
  mpc_ptr = &mpc;

  double frequency;
  ros::param::get("~frequency", frequency);
  ros::Rate rate(frequency);

  ros::Subscriber traversal_subscriber = nh.subscribe("/behavior/traversal_enabled", 100, traversal_bool_callback);

  while (ros::ok()) {
    ros::spinOnce();

    if (traversal_enabled) {
      mpc.calculate_velocity();
    }
    rate.sleep();
  }

}
