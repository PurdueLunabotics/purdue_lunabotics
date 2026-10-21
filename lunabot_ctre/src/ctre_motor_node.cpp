
#include "lunabot_ctre/ctre_motor_node.hpp"

CtreMotorNode::CtreMotorNode()
: Node("ctre_motor_node")
{
  RCLCPP_INFO(this->get_logger(), "CTRE motor node initialized");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<CtreMotorNode>());

  rclcpp::shutdown();
  return 0;
}
