#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "mujoco/mujoco.h"
#include <mujoco/mjvisualize.h>

class Camera {
    private:
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_camera_info_pub;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_camera_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_camera_pub;

        mjvCamera cam;
        int cam_idx;

        sensor_msgs::msg::Image depth_img;
        sensor_msgs::msg::Image rgb_img;
        sensor_msgs::msg::CameraInfo cam_info;
    public:
        Camera(std::string name, std::string frame_id, std::string topic_name, rclcpp::Node* node, mjModel *model);
        void render(mjModel *model, mjData *data, mjvOption *opt, mjvScene *scn, mjrContext *con, builtin_interfaces::msg::Time time);
        uint32_t width() const;
        uint32_t height() const;
};
