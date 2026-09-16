#include "robot.hpp"
#include <algorithm>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

Robot::Robot(std::string ns, mjModel *model, mjData *data, rclcpp::Node *node) {
    this->ns = ns;
    effort_sub = node->create_subscription<lunabot_msgs::msg::RobotEffort>(apply_ns("effort", "/"), 10, [this] (lunabot_msgs::msg::RobotEffort effort) {
        this->effort = effort;
    });
    odom_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(apply_ns("simulation/odom", "/"), 10);

    left_act = Actuator(apply_ns("left", " "), model, data);
    right_act = Actuator(apply_ns("right", " "), model, data);

    odom_pos_sensor = Sensor(apply_ns("odom_pos", "_"), model, data);
    odom_rot_sensor = Sensor(apply_ns("odom_rot", "_"), model, data);

    front_camera = std::make_shared<Camera>(apply_ns("front", " "), apply_ns("d455_front_sim_link", "/"), apply_ns("d455_front", "/"), node, model);
    back_camera = std::make_shared<Camera>(apply_ns("back", " "), apply_ns("d455_back_sim_link", "/"), apply_ns("d455_back", "/"), node, model);
}

void Robot::apply_controls() {
    left_act.ctrl(effort.left_drive / 1000.0);
    right_act.ctrl(effort.right_drive / 1000.0);
}

void Robot::publish_odom(builtin_interfaces::msg::Time time) {
    geometry_msgs::msg::PoseStamped pose;
    mjtNum *pos_data = odom_pos_sensor.get();
    mjtNum *rot_data = odom_rot_sensor.get();
    pose.header.frame_id = apply_ns("odom", "/");
    pose.header.stamp = time;

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
}

void Robot::publish_cameras(mjModel *model, mjData *data, mjvOption *opt, mjvScene *scn, mjrContext *con, builtin_interfaces::msg::Time time) {
    front_camera->render(model, data, opt, scn, con, time);
    back_camera->render(model, data, opt, scn, con, time);
}

std::string Robot::apply_ns(std::string input, std::string delim) {
    if (ns.empty()) {
        return input;
    }
    return ns + delim + input;
}

uint32_t Robot::max_cam_width() const {
    return std::max(front_camera->width(), back_camera->width());
}

uint32_t Robot::max_cam_height() const {
    return std::max(front_camera->height(), back_camera->height());
}
