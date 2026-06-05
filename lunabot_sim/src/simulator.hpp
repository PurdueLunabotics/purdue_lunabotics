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

        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr front_rgb_camera_info_pub;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr front_depth_camera_info_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr front_depth_camera_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr front_rgb_camera_pub;

        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr back_rgb_camera_info_pub;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr back_depth_camera_info_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr back_depth_camera_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr back_rgb_camera_pub;

        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mini_front_rgb_camera_info_pub;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mini_front_depth_camera_info_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mini_front_depth_camera_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mini_front_rgb_camera_pub;

        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mini_back_rgb_camera_info_pub;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mini_back_depth_camera_info_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mini_back_depth_camera_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mini_back_rgb_camera_pub;

        mjvCamera front_cam;
        mjvCamera back_cam;
        mjvCamera mini_front_cam;
        mjvCamera mini_back_cam;

        sensor_msgs::msg::Image front_depth_img;
        sensor_msgs::msg::Image front_rgb_img;
        sensor_msgs::msg::CameraInfo front_cam_info;
        sensor_msgs::msg::Image back_depth_img;
        sensor_msgs::msg::Image back_rgb_img;
        sensor_msgs::msg::CameraInfo back_cam_info;
        sensor_msgs::msg::Image mini_front_depth_img;
        sensor_msgs::msg::Image mini_front_rgb_img;
        sensor_msgs::msg::CameraInfo mini_front_cam_info;
        sensor_msgs::msg::Image mini_back_depth_img;
        sensor_msgs::msg::Image mini_back_rgb_img;
        sensor_msgs::msg::CameraInfo mini_back_cam_info;

        std::vector<unsigned char> row_swap_buf;

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
        int front_cam_idx;
        int back_cam_idx;
        int mini_front_cam_idx;
        int mini_back_cam_idx;

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
