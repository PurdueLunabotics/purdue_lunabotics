#include "main_robot.hpp"

MainRobot::MainRobot(std::string ns, mjModel *model, mjData *data, rclcpp::Node *node): Robot(ns, model, data, node) {
    joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("/simulator/joint_states", 10);
    exc_act = Actuator("excavation", model, data);
    exc_pos_sensor = Sensor("exc_pos", model, data);
    exc_effort_sensor = Sensor("exc_effort", model, data);
}

void MainRobot::apply_controls() {
    Robot::apply_controls();
    exc_act.ctrl(effort.lin_act / 128.0);
}

void MainRobot::publish_odom(builtin_interfaces::msg::Time time) {
    Robot::publish_odom(time);
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = time;
    joint_state.name = { "excavation_joint" };
    joint_state.position = { *exc_pos_sensor.get() };
    joint_state.effort = { *exc_effort_sensor.get() / 20.0 };

    joint_state_pub->publish(joint_state);
}
