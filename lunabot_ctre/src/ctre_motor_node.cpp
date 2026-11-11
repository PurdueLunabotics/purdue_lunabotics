
#include <algorithm>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "ctre/phoenix6/TalonFX.hpp"

class CTREMotorNode : public rclcpp::Node
{
public:
    CTREMotorNode()
        : Node("ctre_motor_node"),
          can_id_(this->declare_parameter<int>("can_id", 1)),
          can_bus_(this->declare_parameter<std::string>("can_bus", "rio")),
          motor_(can_id_, can_bus_)
    {
        command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "motor_command",
            10,
            std::bind(
                &CTREMotorNode::command_callback,
                this,
                std::placeholders::_1
            )
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Initialized TalonFX (ID: %d, bus: %s)",
            can_id_,
            can_bus_.c_str()
        );
    }

private:
    void command_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        double output = std::clamp(msg->data, -1.0, 1.0);

        motor_.SetControl(duty_cycle_request_.WithOutput(output));
    }

    int can_id_;
    std::string can_bus_;

    ctre::phoenix6::hardware::TalonFX motor_;
    ctre::phoenix6::controls::DutyCycleOut duty_cycle_request_{0.0};

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CTREMotorNode>());

    rclcpp::shutdown();
    return 0;
}
