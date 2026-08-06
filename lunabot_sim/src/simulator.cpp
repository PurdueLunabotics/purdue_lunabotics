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

// mujoco specific code taken from https://github.com/google-deepmind/mujoco/blob/main/sample/basic.cc

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
    clock_pub = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

    scene_path = declare_parameter<std::string>("scene_path", "");
    char *error = new char[1024];
    model = mj_loadXML(this->scene_path.c_str(), NULL, error, 1024);
    if (model == NULL) {
        RCLCPP_ERROR(get_logger(), "Failed to load xml: %s", error);
        return;
    }
    data = mj_makeData(model);

    main_bot = std::make_shared<MainRobot>("", model, data, this);
    mini_bot = std::make_shared<Robot>("mini", model, data, this);

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
    int max_width = std::max(main_bot->max_cam_width(), mini_bot->max_cam_width());
    int max_height = std::max(main_bot->max_cam_height(), mini_bot->max_cam_height());
    mjr_resizeOffscreen(max_width, max_height, &con);

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
            main_bot->apply_controls();
            mini_bot->apply_controls();
            // exc_act.ctrl(effort.lin_act / 128.0);
            mj_step2(model, data);
        }

        rosgraph_msgs::msg::Clock clock;
        clock.clock = get_time();
        clock_pub->publish(clock);

        main_bot->publish_odom(get_time());
        mini_bot->publish_odom(get_time());

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(model, data, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        if (frame_counter == 0) {
            mjr_setBuffer(mjFB_OFFSCREEN, &con);
            main_bot->publish_cameras(model, data, &opt, &scn, &con, get_time());
            mini_bot->publish_cameras(model, data, &opt, &scn, &con, get_time());
            mjr_setBuffer(mjFB_WINDOW, &con);
        }
        frame_counter++;
        frame_counter %= 4; // TODO: proper fps counting
                            //
        rclcpp::spin_some(node);

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
