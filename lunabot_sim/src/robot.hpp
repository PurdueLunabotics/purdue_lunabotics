#pragma once

#include "actuator.hpp"
#include "sensor.hpp"
#include "camera.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lunabot_msgs/msg/robot_effort.hpp"

class Robot {
    protected:
        Actuator left_act;
        Actuator right_act;

        Sensor odom_pos_sensor;
        Sensor odom_rot_sensor;

        std::shared_ptr<Camera> front_camera;
        std::shared_ptr<Camera> back_camera;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_pub;
        rclcpp::Subscription<lunabot_msgs::msg::RobotEffort>::SharedPtr effort_sub;
        lunabot_msgs::msg::RobotEffort effort;

        std::string ns;

        std::string apply_ns(std::string input, std::string delim);

    public:
        Robot(std::string ns, mjModel *model, mjData *data, rclcpp::Node *node);
        void apply_controls();
        void publish_odom(builtin_interfaces::msg::Time time);
        void publish_cameras(mjModel *model, mjData *data, mjvOption *opt, mjvScene *scn, mjrContext *con, builtin_interfaces::msg::Time time);

        uint32_t max_cam_width() const;
        uint32_t max_cam_height() const;
};
