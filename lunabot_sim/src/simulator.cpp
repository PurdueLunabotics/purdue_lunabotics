#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"
#include "simulator.hpp"
#include "GLFW/glfw3.h"
#include <cstring>
#include <memory>

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
        }
    }

    for (int i = 0; i < model->nsensor; i++) {
        const char *sensor_name = &model->names[model->name_sensoradr[i]];
        if (strcmp(sensor_name, "odom_pos") == 0) {
            odom_pos_sensor_idx = i;
        } else if (strcmp(sensor_name, "odom_rot") == 0) {
            odom_rot_sensor_idx = i;
        }
    }

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
            mj_step2(model, data);
            rclcpp::spin_some(node);
        }

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(model, data, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

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

    // RCLCPP_INFO(get_logger(), "odom pos num dims: %d", model->sensor_dim[odom_pos_sensor_idx]);
    // RCLCPP_INFO(get_logger(), "odom rot num dims: %d", model->sensor_dim[odom_rot_sensor_idx]);
    //
    // while (rclcpp::ok()) {
    //     mj_step1(model, data);
    //     data->ctrl[left_act_idx] = 1.0;
    //     data->ctrl[right_act_idx] = 1.0;
    //     mj_step2(model, data);
    //     mjtNum *pos_data = &data->sensordata[model->sensor_adr[odom_pos_sensor_idx]];
    //     for (int i = 0; i < model->sensor_dim[odom_pos_sensor_idx]; i++) {
    //         RCLCPP_INFO(get_logger(), "pos data %d: %f", i, pos_data[i]);
    //     }
    // }
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
