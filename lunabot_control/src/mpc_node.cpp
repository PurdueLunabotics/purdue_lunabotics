#include <lunabot_control/mpc.h>

bool traversal_enabled = true;

MPC* mpc_ptr = NULL;

void traversal_bool_callback(const std_msgs::msg::Bool& msg) {
  traversal_enabled = msg.data;

  if (!traversal_enabled & mpc_ptr != NULL) {
    mpc_ptr->publish_velocity(0, 0);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mpc_node");

  MPC mpc;
  mpc_ptr = &mpc;

  double frequency = node->get_parameter("frequency").as_double();
  rclcpp::Rate rate(frequency);
  
  auto traversal_subscriber = node->create_subscription<std_msgs::msg::Bool>(
      "/behavior/traversal_enabled", 10, std::bind(&traversal_bool_callback, *node, _1));

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    if (traversal_enabled) {
      mpc.calculate_velocity();
    }
    rate.sleep();
  }

}
