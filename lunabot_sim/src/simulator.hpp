#include <string>
#include "rclcpp/rclcpp.hpp"
#include "mujoco/mujoco.h"
#include "lunabot_msgs/msg/robot_effort.hpp"
#include "GLFW/glfw3.h"

class SimulatorNode : public rclcpp::Node {
    private:
        std::string scene_path;
        mjModel *model;
        mjData *data;

        mjvCamera cam; // abstract camera
        mjvOption opt; // visualization options
        mjvScene scn; // abstract scene
        mjrContext con; // custom GPU context
        GLFWwindow* window;
        bool button_left = false;
        bool button_middle = false;
        bool button_right =  false;
        double lastx = 0;
        double lasty = 0;
        
        rclcpp::Subscription<lunabot_msgs::msg::RobotEffort>::SharedPtr effort_sub;
        lunabot_msgs::msg::RobotEffort effort;

        int left_act_idx;
        int right_act_idx;
        int odom_pos_sensor_idx;
        int odom_rot_sensor_idx;

    public:
        SimulatorNode();
        void run_loop(rclcpp::Node::SharedPtr node);

        void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);

        void mouse_button(GLFWwindow* window, int button, int act, int mods);

        void mouse_move(GLFWwindow* window, double xpos, double ypos);

        void scroll(GLFWwindow* window, double xoffset, double yoffset);

};
