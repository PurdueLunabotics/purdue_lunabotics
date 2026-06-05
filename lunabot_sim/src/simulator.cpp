#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mujoco/mujoco.h"
#include "simulator.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "GLFW/glfw3.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <mujoco/mjrender.h>
#include <mujoco/mjvisualize.h>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

void global_keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    SimulatorNode *node = (SimulatorNode *) glfwGetWindowUserPointer(window);
    node->keyboard(window, key, scancode, act, mods);
}

void global_mouse_button(GLFWwindow* window, int button, int act, int mods) {
    SimulatorNode *node = (SimulatorNode *) glfwGetWindowUserPointer(window);
    node->mouse_button(window, button, act, mods);
}

void global_mouse_move(GLFWwindow* window, double xpos, double ypos) {
    SimulatorNode *node = (SimulatorNode *) glfwGetWindowUserPointer(window);
    node->mouse_move(window, xpos, ypos);
}

void global_scroll(GLFWwindow* window, double xoffset, double yoffset) {
    SimulatorNode *node = (SimulatorNode *) glfwGetWindowUserPointer(window);
    node->scroll(window, xoffset, yoffset);
}

SimulatorNode::SimulatorNode(): rclcpp::Node("simulator_node") {
    effort_sub = create_subscription<lunabot_msgs::msg::RobotEffort>("/effort", 10, [this] (lunabot_msgs::msg::RobotEffort effort) {
        this->effort = effort;
    });
    mini_effort_sub = create_subscription<lunabot_msgs::msg::RobotEffort>("/mini/effort", 10, [this] (lunabot_msgs::msg::RobotEffort effort) {
        this->mini_effort = effort;
    });
    clock_pub = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    odom_pub = create_publisher<geometry_msgs::msg::PoseStamped>("/simulation/odom", 10);
    mini_odom_pub = create_publisher<geometry_msgs::msg::PoseStamped>("/mini/simulation/odom", 10);
    joint_state_pub = create_publisher<sensor_msgs::msg::JointState>("/simulator/joint_states", 10);

    front_rgb_camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>("/d455_front/color/camera_info", 10);
    front_depth_camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>("/d455_front/aligned_depth_to_color/camera_info", 10);
    front_depth_camera_pub = create_publisher<sensor_msgs::msg::Image>("/d455_front/aligned_depth_to_color/image_raw", 10);
    front_rgb_camera_pub = create_publisher<sensor_msgs::msg::Image>("/d455_front/color/image_raw", 10);

    back_rgb_camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>("/d455_back/color/camera_info", 10);
    back_depth_camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>("/d455_back/aligned_depth_to_color/camera_info", 10);
    back_depth_camera_pub = create_publisher<sensor_msgs::msg::Image>("/d455_back/aligned_depth_to_color/image_raw", 10);
    back_rgb_camera_pub = create_publisher<sensor_msgs::msg::Image>("/d455_back/color/image_raw", 10);

    mini_front_rgb_camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>("/mini/d455_front/color/camera_info", 10);
    mini_front_depth_camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>("/mini/d455_front/aligned_depth_to_color/camera_info", 10);
    mini_front_depth_camera_pub = create_publisher<sensor_msgs::msg::Image>("/mini/d455_front/aligned_depth_to_color/image_raw", 10);
    mini_front_rgb_camera_pub = create_publisher<sensor_msgs::msg::Image>("/mini/d455_front/color/image_raw", 10);

    mini_back_rgb_camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>("/mini/d455_back/color/camera_info", 10);
    mini_back_depth_camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>("/mini/d455_back/aligned_depth_to_color/camera_info", 10);
    mini_back_depth_camera_pub = create_publisher<sensor_msgs::msg::Image>("/mini/d455_back/aligned_depth_to_color/image_raw", 10);
    mini_back_rgb_camera_pub = create_publisher<sensor_msgs::msg::Image>("/mini/d455_back/color/image_raw", 10);

    scene_path = declare_parameter<std::string>("scene_path", "/luna_ws/src/purdue_lunabotics/lunabot_sim/mujoco/scene.xml");
    char *error = new char[1024];
    model = mj_loadXML(this->scene_path.c_str(), NULL, error, 1024);
    if (model == NULL) {
        RCLCPP_ERROR(get_logger(), "Failed to load xml: %s", error);
        return;
    }
    data = mj_makeData(model);

    for (int i = 0; i < model->nu; i++) {
        const char *act_name = &model->names[model->name_actuatoradr[i]];
        if (strcmp(act_name, "left") == 0) {
            left_act_idx = i;
        } else if (strcmp(act_name, "right") == 0) {
            right_act_idx = i;
        } else if (strcmp(act_name, "excavation") == 0) {
            exc_act_idx = i;
        } else if (strcmp(act_name, "mini left") == 0) {
            mini_left_act_idx = i;
        } else if (strcmp(act_name, "mini right") == 0) {
            mini_right_act_idx = i;
        }
    }

    for (int i = 0; i < model->nsensor; i++) {
        const char *sensor_name = &model->names[model->name_sensoradr[i]];
        if (strcmp(sensor_name, "odom_pos") == 0) {
            odom_pos_sensor_idx = i;
        } else if (strcmp(sensor_name, "odom_rot") == 0) {
            odom_rot_sensor_idx = i;
        } else if (strcmp(sensor_name, "exc_pos") == 0) {
            exc_pos_sensor_idx = i;
        } else if (strcmp(sensor_name, "exc_effort") == 0) {
            exc_effort_sensor_idx = i;
        } else if (strcmp(sensor_name, "mini_odom_pos") == 0) {
            mini_odom_pos_sensor_idx = i;
        } else if (strcmp(sensor_name, "mini_odom_rot") == 0) {
            mini_odom_rot_sensor_idx = i;
        }
    }

    for (int i = 0; i < model->ncam; i++) {
        const char *cam_name = &model->names[model->name_camadr[i]];
        if (strcmp(cam_name, "front") == 0) {
            front_cam_idx = i;
        } else if (strcmp(cam_name, "back") == 0) {
            back_cam_idx = i;
        } else if (strcmp(cam_name, "mini front") == 0) {
            mini_front_cam_idx = i;
        } else if (strcmp(cam_name, "mini back") == 0) {
            mini_back_cam_idx = i;
        }
    }

    front_cam.type = mjCAMERA_FIXED;
    front_cam.fixedcamid = front_cam_idx;
    back_cam.type = mjCAMERA_FIXED;
    back_cam.fixedcamid = back_cam_idx;
    mini_front_cam.type = mjCAMERA_FIXED;
    mini_front_cam.fixedcamid = mini_front_cam_idx;
    mini_back_cam.type = mjCAMERA_FIXED;
    mini_back_cam.fixedcamid = mini_back_cam_idx;

    int *front_cam_res = &model->cam_resolution[front_cam_idx * 2];

    front_rgb_img.width = front_cam_res[0];
    front_rgb_img.height = front_cam_res[1];
    front_rgb_img.step = front_rgb_img.width * 3;
    front_rgb_img.encoding = sensor_msgs::image_encodings::RGB8;
    front_rgb_img.data.resize(front_rgb_img.width * front_rgb_img.height * 3);
    front_rgb_img.header.frame_id = "d455_front_sim_link";
    front_depth_img.width = front_cam_res[0];
    front_depth_img.height = front_cam_res[1];
    front_depth_img.step = front_rgb_img.width * sizeof(float);
    front_depth_img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    front_depth_img.data.resize(front_rgb_img.width * front_rgb_img.height * sizeof(float));
    front_depth_img.header.frame_id = "d455_front_sim_link";

    front_cam_info.header.frame_id = "d455_front_sim_link";
    front_cam_info.width = front_cam_res[0];
    front_cam_info.height = front_cam_res[1];
    front_cam_info.distortion_model = "plumb_bob";
    front_cam_info.k.fill(0.0);
    front_cam_info.r.fill(0.0);
    front_cam_info.p.fill(0.0);
    front_cam_info.d.resize(5, 0.0);

    double focal_scaling = (1.0 / std::tan((model->cam_fovy[front_cam_idx] * M_PI / 180.0) / 2.0)) * front_cam_res[1] / 2.0;
    front_cam_info.k[0] = front_cam_info.p[0] = focal_scaling;
    front_cam_info.k[2] = front_cam_info.p[2] = static_cast<double>(front_cam_res[0]) / 2.0;
    front_cam_info.k[4] = front_cam_info.p[5] = focal_scaling;
    front_cam_info.k[5] = front_cam_info.p[6] = static_cast<double>(front_cam_res[1]) / 2.0;
    front_cam_info.k[8] = front_cam_info.p[10] = 1.0;

    int *back_cam_res = &model->cam_resolution[back_cam_idx * 2];

    back_rgb_img.width = back_cam_res[0];
    back_rgb_img.height = back_cam_res[1];
    back_rgb_img.step = back_rgb_img.width * 3;
    back_rgb_img.encoding = sensor_msgs::image_encodings::RGB8;
    back_rgb_img.data.resize(back_rgb_img.width * back_rgb_img.height * 3);
    back_rgb_img.header.frame_id = "d455_back_sim_link";
    back_depth_img.width = back_cam_res[0];
    back_depth_img.height = back_cam_res[1];
    back_depth_img.step = back_rgb_img.width * sizeof(float);
    back_depth_img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    back_depth_img.data.resize(back_rgb_img.width * back_rgb_img.height * sizeof(float));
    back_depth_img.header.frame_id = "d455_back_sim_link";

    back_cam_info.header.frame_id = "d455_back_sim_link";
    back_cam_info.width = back_cam_res[0];
    back_cam_info.height = back_cam_res[1];
    back_cam_info.distortion_model = "plumb_bob";
    back_cam_info.k.fill(0.0);
    back_cam_info.r.fill(0.0);
    back_cam_info.p.fill(0.0);
    back_cam_info.d.resize(5, 0.0);

    focal_scaling = (1.0 / std::tan((model->cam_fovy[back_cam_idx] * M_PI / 180.0) / 2.0)) * back_cam_res[1] / 2.0;
    back_cam_info.k[0] = back_cam_info.p[0] = focal_scaling;
    back_cam_info.k[2] = back_cam_info.p[2] = static_cast<double>(back_cam_res[0]) / 2.0;
    back_cam_info.k[4] = back_cam_info.p[5] = focal_scaling;
    back_cam_info.k[5] = back_cam_info.p[6] = static_cast<double>(back_cam_res[1]) / 2.0;
    back_cam_info.k[8] = back_cam_info.p[10] = 1.0;

    int *mini_front_cam_res = &model->cam_resolution[mini_front_cam_idx * 2];

    mini_front_rgb_img.width = mini_front_cam_res[0];
    mini_front_rgb_img.height = mini_front_cam_res[1];
    mini_front_rgb_img.step = mini_front_rgb_img.width * 3;
    mini_front_rgb_img.encoding = sensor_msgs::image_encodings::RGB8;
    mini_front_rgb_img.data.resize(mini_front_rgb_img.width * mini_front_rgb_img.height * 3);
    mini_front_rgb_img.header.frame_id = "mini/d455_front_sim_link";
    mini_front_depth_img.width = mini_front_cam_res[0];
    mini_front_depth_img.height = mini_front_cam_res[1];
    mini_front_depth_img.step = mini_front_rgb_img.width * sizeof(float);
    mini_front_depth_img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    mini_front_depth_img.data.resize(mini_front_rgb_img.width * mini_front_rgb_img.height * sizeof(float));
    mini_front_depth_img.header.frame_id = "mini/d455_front_sim_link";

    mini_front_cam_info.header.frame_id = "mini/d455_front_sim_link";
    mini_front_cam_info.width = mini_front_cam_res[0];
    mini_front_cam_info.height = mini_front_cam_res[1];
    mini_front_cam_info.distortion_model = "plumb_bob";
    mini_front_cam_info.k.fill(0.0);
    mini_front_cam_info.r.fill(0.0);
    mini_front_cam_info.p.fill(0.0);
    mini_front_cam_info.d.resize(5, 0.0);

    focal_scaling = (1.0 / std::tan((model->cam_fovy[mini_front_cam_idx] * M_PI / 180.0) / 2.0)) * mini_front_cam_res[1] / 2.0;
    mini_front_cam_info.k[0] = mini_front_cam_info.p[0] = focal_scaling;
    mini_front_cam_info.k[2] = mini_front_cam_info.p[2] = static_cast<double>(mini_front_cam_res[0]) / 2.0;
    mini_front_cam_info.k[4] = mini_front_cam_info.p[5] = focal_scaling;
    mini_front_cam_info.k[5] = mini_front_cam_info.p[6] = static_cast<double>(mini_front_cam_res[1]) / 2.0;
    mini_front_cam_info.k[8] = mini_front_cam_info.p[10] = 1.0;

    int *mini_back_cam_res = &model->cam_resolution[mini_back_cam_idx * 2];

    mini_back_rgb_img.width = mini_back_cam_res[0];
    mini_back_rgb_img.height = mini_back_cam_res[1];
    mini_back_rgb_img.step = mini_back_rgb_img.width * 3;
    mini_back_rgb_img.encoding = sensor_msgs::image_encodings::RGB8;
    mini_back_rgb_img.data.resize(mini_back_rgb_img.width * mini_back_rgb_img.height * 3);
    mini_back_rgb_img.header.frame_id = "mini/d455_back_sim_link";
    mini_back_depth_img.width = mini_back_cam_res[0];
    mini_back_depth_img.height = mini_back_cam_res[1];
    mini_back_depth_img.step = mini_back_rgb_img.width * sizeof(float);
    mini_back_depth_img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    mini_back_depth_img.data.resize(mini_back_rgb_img.width * mini_back_rgb_img.height * sizeof(float));
    mini_back_depth_img.header.frame_id = "mini/d455_back_sim_link";

    mini_back_cam_info.header.frame_id = "mini/d455_back_sim_link";
    mini_back_cam_info.width = mini_back_cam_res[0];
    mini_back_cam_info.height = mini_back_cam_res[1];
    mini_back_cam_info.distortion_model = "plumb_bob";
    mini_back_cam_info.k.fill(0.0);
    mini_back_cam_info.r.fill(0.0);
    mini_back_cam_info.p.fill(0.0);
    mini_back_cam_info.d.resize(5, 0.0);

    focal_scaling = (1.0 / std::tan((model->cam_fovy[mini_back_cam_idx] * M_PI / 180.0) / 2.0)) * mini_back_cam_res[1] / 2.0;
    mini_back_cam_info.k[0] = mini_back_cam_info.p[0] = focal_scaling;
    mini_back_cam_info.k[2] = mini_back_cam_info.p[2] = static_cast<double>(mini_back_cam_res[0]) / 2.0;
    mini_back_cam_info.k[4] = mini_back_cam_info.p[5] = focal_scaling;
    mini_back_cam_info.k[5] = mini_back_cam_info.p[6] = static_cast<double>(mini_back_cam_res[1]) / 2.0;
    mini_back_cam_info.k[8] = mini_back_cam_info.p[10] = 1.0;

    if (!glfwInit()) {
        RCLCPP_ERROR(get_logger(), "Could not initialize GLFW");
        return;
    }

    // create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1200, 900, "MuJoCo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    glfwSetWindowUserPointer(window, this);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(model, &scn, 2000);
    mjr_makeContext(model, &con, mjFONTSCALE_150);
    int max_width = std::max(std::max(back_cam_info.width, front_cam_info.width), std::max(mini_back_cam_info.width, mini_front_cam_info.width));
    int max_height = std::max(std::max(back_cam_info.height, front_cam_info.height), std::max(mini_back_cam_info.height, mini_front_cam_info.height));
    mjr_resizeOffscreen(max_width, max_height, &con);
    row_swap_buf.resize(max_width * 3);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, global_keyboard);
    glfwSetCursorPosCallback(window, global_mouse_move);
    glfwSetMouseButtonCallback(window, global_mouse_button);
    glfwSetScrollCallback(window, global_scroll);
}

void SimulatorNode::run_loop(rclcpp::Node::SharedPtr node) {
    // run main loop, target real-time simulation and 60 fps rendering
    while (rclcpp::ok()) {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = data->time;
        while (data->time - simstart < 1.0/60.0) {
            mj_step1(model, data);
            data->ctrl[left_act_idx] = effort.left_drive / 1000.0;
            data->ctrl[right_act_idx] = effort.right_drive / 1000.0;
            data->ctrl[mini_left_act_idx] = mini_effort.left_drive / 1000.0;
            data->ctrl[mini_right_act_idx] = mini_effort.right_drive / 1000.0;
            data->ctrl[exc_act_idx] = effort.lin_act / 128.0;
            mj_step2(model, data);

            rosgraph_msgs::msg::Clock clock;
            clock.clock = get_time();
            clock_pub->publish(clock);

            rclcpp::spin_some(node);
        }
        geometry_msgs::msg::PoseStamped pose;
        mjtNum *pos_data = &data->sensordata[model->sensor_adr[odom_pos_sensor_idx]];
        mjtNum *rot_data = &data->sensordata[model->sensor_adr[odom_rot_sensor_idx]];
        pose.header.frame_id = "odom";
        pose.header.stamp = get_time();

        tf2::Quaternion original(rot_data[3], rot_data[0], rot_data[1], rot_data[2]);
        original.normalize();

        tf2::Quaternion transform;
        transform.setRPY(0.0, 3.14159, 0.0);
        transform.normalize();
        tf2::Quaternion final = transform * original;
        final.normalize();

        pose.pose.position.x = pos_data[0];
        pose.pose.position.y = pos_data[1];
        pose.pose.position.z = pos_data[2] - 0.3;
        pose.pose.orientation = tf2::toMsg(final);
        odom_pub->publish(pose);

        pos_data = &data->sensordata[model->sensor_adr[mini_odom_pos_sensor_idx]];
        rot_data = &data->sensordata[model->sensor_adr[mini_odom_rot_sensor_idx]];
        pose.header.frame_id = "mini/odom";

        original.setValue(rot_data[3], rot_data[0], rot_data[1], rot_data[2]);
        original.normalize();

        final = transform * original;
        final.normalize();

        pose.pose.position.x = pos_data[0];
        pose.pose.position.y = pos_data[1];
        pose.pose.position.z = pos_data[2] - 0.15;
        pose.pose.orientation = tf2::toMsg(final);
        mini_odom_pub->publish(pose);

        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = get_time();
        joint_state.name = { "excavation_joint" };
        joint_state.position = { data->sensordata[model->sensor_adr[exc_pos_sensor_idx]] };
        joint_state.effort = { data->sensordata[model->sensor_adr[exc_effort_sensor_idx]] / 20.0 };

        joint_state_pub->publish(joint_state);

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(model, data, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        if (frame_counter == 0) {
            mjrRect front_viewport = {0, 0, (int) front_rgb_img.width, (int) front_rgb_img.height};
            mjrRect back_viewport = {0, 0, (int) back_rgb_img.width, (int) back_rgb_img.height};
            mjrRect mini_front_viewport = {0, 0, (int) mini_front_rgb_img.width, (int) mini_front_rgb_img.height};
            mjrRect mini_back_viewport = {0, 0, (int) mini_back_rgb_img.width, (int) mini_back_rgb_img.height};
            mjr_setBuffer(mjFB_OFFSCREEN, &con);
            mjv_updateScene(model, data, &opt, NULL, &front_cam, mjCAT_ALL, &scn);
            mjr_render(front_viewport, &scn, &con);
            float *depth_data = (float *) front_depth_img.data.data();
            mjr_readPixels(front_rgb_img.data.data(), depth_data, front_viewport, &con);
            float near = (float) model->vis.map.znear * model->stat.extent;
            float far = (float) model->vis.map.zfar * model->stat.extent;
            float depth_scale = 1.0f - near / far;
            for (unsigned int h = 0; h < front_depth_img.height / 2; h++) {
                for (unsigned int w = 0; w < front_depth_img.width; w++) {
                    unsigned int idx = h * front_depth_img.width + w;
                    unsigned int flipped_idx = (front_depth_img.height - 1 - h) * front_depth_img.width + w;
                    depth_data[idx] = near / (1.0f - depth_data[idx] * (depth_scale));
                    depth_data[flipped_idx] = near / (1.0f - depth_data[flipped_idx] * (depth_scale));
                    float temp = depth_data[idx];
                    depth_data[idx] = depth_data[flipped_idx];
                    depth_data[flipped_idx] = temp;
                }
            }

            int row_size = front_rgb_img.width * 3;
            for (unsigned int h = 0; h < front_rgb_img.height / 2; h++) {
                unsigned int flipped_h = front_rgb_img.height - 1 - h;
                std::memcpy(row_swap_buf.data(), front_rgb_img.data.data() + h * row_size, row_size);
                std::memcpy(front_rgb_img.data.data() + h * row_size, front_rgb_img.data.data() + flipped_h * row_size, row_size);
                std::memcpy(front_rgb_img.data.data() + flipped_h * row_size, row_swap_buf.data(), row_size);
            }

            front_rgb_img.header.stamp = get_time();
            front_depth_img.header.stamp = get_time();
            front_cam_info.header.stamp = get_time();

            front_rgb_camera_pub->publish(front_rgb_img);
            front_rgb_camera_info_pub->publish(front_cam_info);
            front_depth_camera_pub->publish(front_depth_img);
            front_depth_camera_info_pub->publish(front_cam_info);

            mjv_updateScene(model, data, &opt, NULL, &back_cam, mjCAT_ALL, &scn);
            mjr_render(back_viewport, &scn, &con);
            depth_data = (float *) back_depth_img.data.data();
            mjr_readPixels(back_rgb_img.data.data(), depth_data, back_viewport, &con);
            near = (float) model->vis.map.znear * model->stat.extent;
            far = (float) model->vis.map.zfar * model->stat.extent;
            depth_scale = 1.0f - near / far;
            for (unsigned int h = 0; h < back_depth_img.height / 2; h++) {
                for (unsigned int w = 0; w < back_depth_img.width; w++) {
                    unsigned int idx = h * back_depth_img.width + w;
                    unsigned int flipped_idx = (back_depth_img.height - 1 - h) * back_depth_img.width + w;
                    depth_data[idx] = near / (1.0f - depth_data[idx] * (depth_scale));
                    depth_data[flipped_idx] = near / (1.0f - depth_data[flipped_idx] * (depth_scale));
                    float temp = depth_data[idx];
                    depth_data[idx] = depth_data[flipped_idx];
                    depth_data[flipped_idx] = temp;
                }
            }

            row_size = back_rgb_img.width * 3;
            for (unsigned int h = 0; h < back_rgb_img.height / 2; h++) {
                unsigned int flipped_h = back_rgb_img.height - 1 - h;
                std::memcpy(row_swap_buf.data(), back_rgb_img.data.data() + h * row_size, row_size);
                std::memcpy(back_rgb_img.data.data() + h * row_size, back_rgb_img.data.data() + flipped_h * row_size, row_size);
                std::memcpy(back_rgb_img.data.data() + flipped_h * row_size, row_swap_buf.data(), row_size);
            }

            back_rgb_img.header.stamp = get_time();
            back_depth_img.header.stamp = get_time();
            back_cam_info.header.stamp = get_time();

            back_rgb_camera_pub->publish(back_rgb_img);
            back_rgb_camera_info_pub->publish(back_cam_info);
            back_depth_camera_pub->publish(back_depth_img);
            back_depth_camera_info_pub->publish(back_cam_info);

            mjv_updateScene(model, data, &opt, NULL, &mini_front_cam, mjCAT_ALL, &scn);
            mjr_render(mini_front_viewport, &scn, &con);
            depth_data = (float *) mini_front_depth_img.data.data();
            mjr_readPixels(mini_front_rgb_img.data.data(), depth_data, mini_front_viewport, &con);
            near = (float) model->vis.map.znear * model->stat.extent;
            far = (float) model->vis.map.zfar * model->stat.extent;
            depth_scale = 1.0f - near / far;
            for (unsigned int h = 0; h < mini_front_depth_img.height / 2; h++) {
                for (unsigned int w = 0; w < mini_front_depth_img.width; w++) {
                    unsigned int idx = h * mini_front_depth_img.width + w;
                    unsigned int flipped_idx = (mini_front_depth_img.height - 1 - h) * mini_front_depth_img.width + w;
                    depth_data[idx] = near / (1.0f - depth_data[idx] * (depth_scale));
                    depth_data[flipped_idx] = near / (1.0f - depth_data[flipped_idx] * (depth_scale));
                    float temp = depth_data[idx];
                    depth_data[idx] = depth_data[flipped_idx];
                    depth_data[flipped_idx] = temp;
                }
            }

            row_size = mini_front_rgb_img.width * 3;
            for (unsigned int h = 0; h < mini_front_rgb_img.height / 2; h++) {
                unsigned int flipped_h = mini_front_rgb_img.height - 1 - h;
                std::memcpy(row_swap_buf.data(), mini_front_rgb_img.data.data() + h * row_size, row_size);
                std::memcpy(mini_front_rgb_img.data.data() + h * row_size, mini_front_rgb_img.data.data() + flipped_h * row_size, row_size);
                std::memcpy(mini_front_rgb_img.data.data() + flipped_h * row_size, row_swap_buf.data(), row_size);
            }

            mini_front_rgb_img.header.stamp = get_time();
            mini_front_depth_img.header.stamp = get_time();
            mini_front_cam_info.header.stamp = get_time();

            mini_front_rgb_camera_pub->publish(mini_front_rgb_img);
            mini_front_rgb_camera_info_pub->publish(mini_front_cam_info);
            mini_front_depth_camera_pub->publish(mini_front_depth_img);
            mini_front_depth_camera_info_pub->publish(mini_front_cam_info);

            mjv_updateScene(model, data, &opt, NULL, &mini_back_cam, mjCAT_ALL, &scn);
            mjr_render(mini_back_viewport, &scn, &con);
            depth_data = (float *) mini_back_depth_img.data.data();
            mjr_readPixels(mini_back_rgb_img.data.data(), depth_data, mini_back_viewport, &con);
            near = (float) model->vis.map.znear * model->stat.extent;
            far = (float) model->vis.map.zfar * model->stat.extent;
            depth_scale = 1.0f - near / far;
            for (unsigned int h = 0; h < mini_back_depth_img.height / 2; h++) {
                for (unsigned int w = 0; w < mini_back_depth_img.width; w++) {
                    unsigned int idx = h * mini_back_depth_img.width + w;
                    unsigned int flipped_idx = (mini_back_depth_img.height - 1 - h) * mini_back_depth_img.width + w;
                    depth_data[idx] = near / (1.0f - depth_data[idx] * (depth_scale));
                    depth_data[flipped_idx] = near / (1.0f - depth_data[flipped_idx] * (depth_scale));
                    float temp = depth_data[idx];
                    depth_data[idx] = depth_data[flipped_idx];
                    depth_data[flipped_idx] = temp;
                }
            }

            row_size = mini_back_rgb_img.width * 3;
            for (unsigned int h = 0; h < mini_back_rgb_img.height / 2; h++) {
                unsigned int flipped_h = mini_back_rgb_img.height - 1 - h;
                std::memcpy(row_swap_buf.data(), mini_back_rgb_img.data.data() + h * row_size, row_size);
                std::memcpy(mini_back_rgb_img.data.data() + h * row_size, mini_back_rgb_img.data.data() + flipped_h * row_size, row_size);
                std::memcpy(mini_back_rgb_img.data.data() + flipped_h * row_size, row_swap_buf.data(), row_size);
            }

            mini_back_rgb_img.header.stamp = get_time();
            mini_back_depth_img.header.stamp = get_time();
            mini_back_cam_info.header.stamp = get_time();

            mini_back_rgb_camera_pub->publish(mini_back_rgb_img);
            mini_back_rgb_camera_info_pub->publish(mini_back_cam_info);
            mini_back_depth_camera_pub->publish(mini_back_depth_img);
            mini_back_depth_camera_info_pub->publish(mini_back_cam_info);

            mjr_setBuffer(mjFB_WINDOW, &con);
        }
        frame_counter++;
        frame_counter %= 4; // TODO: proper fps counting

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data
    mj_deleteData(data);
    mj_deleteModel(model);
}

builtin_interfaces::msg::Time SimulatorNode::get_time() {
    builtin_interfaces::msg::Time time;

    float seconds;
    float fractional = std::modf(data->time, &seconds);

    time.sec = (int) seconds;
    time.nanosec = (int) (fractional * 1e9);
    return time;
}

// keyboard callback
void SimulatorNode::keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    // backspace: reset simulation
    if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
        mj_resetData(model, data);
        mj_forward(model, data);
    }
}


// mouse button callback
void SimulatorNode::mouse_button(GLFWwindow* window, int button, int act, int mods) {
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void SimulatorNode::mouse_move(GLFWwindow* window, double xpos, double ypos) {
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right) {
        return;
    }

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
            glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (button_left) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    // move camera
    mjv_moveCamera(model, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void SimulatorNode::scroll(GLFWwindow* window, double xoffset, double yoffset) {
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(model, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<SimulatorNode> node = std::make_shared<SimulatorNode>();
  node->run_loop(node);
  rclcpp::shutdown();
  return 0;
}
