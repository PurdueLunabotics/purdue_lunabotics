#include "camera.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include <vector>

static std::vector<unsigned char> row_swap_buf;

// very heavily inspired by https://github.com/ros-controls/mujoco_ros2_control

Camera::Camera(std::string name, std::string frame_id, std::string topic_name, rclcpp::Node* node, mjModel *model) {
    rgb_camera_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>("/" + topic_name + "/color/camera_info", 10);
    depth_camera_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>("/" + topic_name + "/aligned_depth_to_color/camera_info", 10);
    depth_camera_pub = node->create_publisher<sensor_msgs::msg::Image>("/" + topic_name + "/aligned_depth_to_color/image_raw", 10);
    rgb_camera_pub = node->create_publisher<sensor_msgs::msg::Image>("/" + topic_name + "/color/image_raw", 10);

    for (int i = 0; i < model->ncam; i++) {
        const char *cam_name = &model->names[model->name_camadr[i]];
        if (strcmp(cam_name, name.c_str()) == 0) {
            cam_idx = i;
        }
    }

    cam.type = mjCAMERA_FIXED;
    cam.fixedcamid = cam_idx;

    int *cam_res = &model->cam_resolution[cam_idx * 2];

    rgb_img.width = cam_res[0];
    rgb_img.height = cam_res[1];
    rgb_img.step = rgb_img.width * 3;
    rgb_img.encoding = sensor_msgs::image_encodings::RGB8;
    rgb_img.data.resize(rgb_img.width * rgb_img.height * 3);
    rgb_img.header.frame_id = frame_id;
    depth_img.width = cam_res[0];
    depth_img.height = cam_res[1];
    depth_img.step = rgb_img.width * sizeof(float);
    depth_img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    depth_img.data.resize(rgb_img.width * rgb_img.height * sizeof(float));
    depth_img.header.frame_id = frame_id;

    cam_info.header.frame_id = frame_id;
    cam_info.width = cam_res[0];
    cam_info.height = cam_res[1];
    cam_info.distortion_model = "plumb_bob";
    cam_info.k.fill(0.0);
    cam_info.r.fill(0.0);
    cam_info.p.fill(0.0);
    cam_info.d.resize(5, 0.0);

    double focal_scaling = (1.0 / std::tan((model->cam_fovy[cam_idx] * M_PI / 180.0) / 2.0)) * cam_res[1] / 2.0;
    cam_info.k[0] = cam_info.p[0] = focal_scaling;
    cam_info.k[2] = cam_info.p[2] = static_cast<double>(cam_res[0]) / 2.0;
    cam_info.k[4] = cam_info.p[5] = focal_scaling;
    cam_info.k[5] = cam_info.p[6] = static_cast<double>(cam_res[1]) / 2.0;
    cam_info.k[8] = cam_info.p[10] = 1.0;

    if (row_swap_buf.capacity() < cam_info.width * 3) {
        row_swap_buf.resize(cam_info.width * 3);
    }
}

void Camera::render(mjModel *model, mjData *data, mjvOption *opt, mjvScene *scn, mjrContext *con, builtin_interfaces::msg::Time time) {
    mjrRect viewport = {0, 0, (int) rgb_img.width, (int) rgb_img.height};
    mjv_updateScene(model, data, opt, NULL, &cam, mjCAT_ALL, scn);
    mjr_render(viewport, scn, con);
    float *depth_data = (float *) depth_img.data.data();
    mjr_readPixels(rgb_img.data.data(), depth_data, viewport, con);
    float near = (float) model->vis.map.znear * model->stat.extent;
    float far = (float) model->vis.map.zfar * model->stat.extent;
    float depth_scale = 1.0f - near / far;
    for (unsigned int h = 0; h < depth_img.height / 2; h++) {
        for (unsigned int w = 0; w < depth_img.width; w++) {
            unsigned int idx = h * depth_img.width + w;
            unsigned int flipped_idx = (depth_img.height - 1 - h) * depth_img.width + w;
            depth_data[idx] = near / (1.0f - depth_data[idx] * (depth_scale));
            depth_data[flipped_idx] = near / (1.0f - depth_data[flipped_idx] * (depth_scale));
            float temp = depth_data[idx];
            depth_data[idx] = depth_data[flipped_idx];
            depth_data[flipped_idx] = temp;
        }
    }

    int row_size = rgb_img.width * 3;
    for (unsigned int h = 0; h < rgb_img.height / 2; h++) {
        unsigned int flipped_h = rgb_img.height - 1 - h;
        std::memcpy(row_swap_buf.data(), rgb_img.data.data() + h * row_size, row_size);
        std::memcpy(rgb_img.data.data() + h * row_size, rgb_img.data.data() + flipped_h * row_size, row_size);
        std::memcpy(rgb_img.data.data() + flipped_h * row_size, row_swap_buf.data(), row_size);
    }

    rgb_img.header.stamp = time;
    depth_img.header.stamp = time;
    cam_info.header.stamp = time;

    rgb_camera_pub->publish(rgb_img);
    rgb_camera_info_pub->publish(cam_info);
    depth_camera_pub->publish(depth_img);
    depth_camera_info_pub->publish(cam_info);
}

uint32_t Camera::width() const {
    return cam_info.width;
}

uint32_t Camera::height() const {
    return cam_info.height;
}
