#pragma once

#include "actuator.hpp"
#include "robot.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

class MainRobot: public Robot {
    private:
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;

        Actuator exc_act;

        Sensor exc_pos_sensor;
        Sensor exc_effort_sensor;

    public:
        MainRobot(std::string ns, mjModel *model, mjData *data, rclcpp::Node *node);
        void apply_controls();
        void publish_odom(builtin_interfaces::msg::Time time);
};
