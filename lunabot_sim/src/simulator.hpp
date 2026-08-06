#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "mujoco/mujoco.h"
#include "lunabot_msgs/msg/robot_effort.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "GLFW/glfw3.h"
#include "sensor_msgs/msg/joint_state.hpp"

#include "robot.hpp"
#include "main_robot.hpp"

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
        
        rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub;
        
        std::shared_ptr<MainRobot> main_bot;
        std::shared_ptr<Robot> mini_bot;

        int frame_counter = 0;

    public:
        SimulatorNode();
        
        void run_loop(rclcpp::Node::SharedPtr node);
        builtin_interfaces::msg::Time get_time();
        void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
        void mouse_button(GLFWwindow* window, int button, int act, int mods);
        void mouse_move(GLFWwindow* window, double xpos, double ypos);
        void scroll(GLFWwindow* window, double xoffset, double yoffset);

};
