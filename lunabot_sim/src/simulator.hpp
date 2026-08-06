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

#include "camera.hpp"

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
        rclcpp::Subscription<lunabot_msgs::msg::RobotEffort>::SharedPtr mini_effort_sub;
        rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mini_odom_pub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
        lunabot_msgs::msg::RobotEffort effort;
        lunabot_msgs::msg::RobotEffort mini_effort;

        std::shared_ptr<Camera> front_camera;
        std::shared_ptr<Camera> back_camera;
        std::shared_ptr<Camera> mini_front_camera;
        std::shared_ptr<Camera> mini_back_camera;

        int left_act_idx;
        int right_act_idx;
        int mini_left_act_idx;
        int mini_right_act_idx;
        int exc_act_idx;
        int odom_pos_sensor_idx;
        int odom_rot_sensor_idx;
        int mini_odom_pos_sensor_idx;
        int mini_odom_rot_sensor_idx;
        int exc_pos_sensor_idx;
        int exc_effort_sensor_idx;

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
