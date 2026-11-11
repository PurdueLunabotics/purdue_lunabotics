#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lunabot_msgs/msg/robot_effort.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include <rclcpp/timer.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <vector>

float limit_accel(float current, float target, float accel, float dt) {
    if (std::fabs(current - target) < accel * dt) {
        return target;
    }

    if (current < target) {
        return current + accel * dt;
    } else {
        return current - accel * dt;
    }
}

class SimpleSimulator: public rclcpp::Node {
    private:
        rclcpp::Subscription<lunabot_msgs::msg::RobotEffort>::SharedPtr effort_sub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
        rclcpp::TimerBase::SharedPtr timer;
        tf2_ros::TransformBroadcaster tf_pub;
        lunabot_msgs::msg::RobotEffort effort;
        tf2::Quaternion rotation = tf2::Quaternion::getIdentity();
        tf2::Vector3 position;
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        sensor_msgs::msg::JointState joint_states;
        float current_left_vel = 0;
        float current_right_vel = 0;
        float current_exc_vel = 0;

        float wheel_radius;
        float wheelbase_width; // half
        float gear_ratio;
        float drive_accel;
        float exc_accel;

        float left_rot_amount = 0;
        float right_rot_amount = 0;
        float exc_pos = 0;

    public:
        SimpleSimulator(): rclcpp::Node("simple_simulator"), tf_pub(this) {
            geometry_msgs::msg::TransformStamped map_to_odom_tf;
            map_to_odom_tf.header.frame_id = "map";
            map_to_odom_tf.child_frame_id = "odom";
            transforms.push_back(map_to_odom_tf);

            geometry_msgs::msg::TransformStamped odom_to_base_tf;
            odom_to_base_tf.header.frame_id = "odom";
            odom_to_base_tf.child_frame_id = "base_link";
            transforms.push_back(odom_to_base_tf);

            joint_states.name.push_back("left_front_rev");
            joint_states.name.push_back("left_back_rev");
            joint_states.name.push_back("right_front_rev");
            joint_states.name.push_back("right_back_rev");
            joint_states.name.push_back("excavation_joint");
            joint_states.position.push_back(0);
            joint_states.position.push_back(0);
            joint_states.position.push_back(0);
            joint_states.position.push_back(0);
            joint_states.position.push_back(0);

            wheel_radius = declare_parameter("wheel_radius", 0.2);
            wheelbase_width = declare_parameter("wheelbase_width", 0.32);
            gear_ratio = declare_parameter("gear_ratio", 50.0);
            drive_accel = declare_parameter("drive_accel", 6000.0);
            exc_accel = declare_parameter("exc_accel", 500.0);

            effort_sub = create_subscription<lunabot_msgs::msg::RobotEffort>("effort", 10, [this] (lunabot_msgs::msg::RobotEffort effort) {
                this->effort = effort;
            });
            pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("position", 10);
            joint_state_pub = create_publisher<sensor_msgs::msg::JointState>("simulator/joint_states", 10);
            timer = create_timer(std::chrono::milliseconds(50), [this] () {
                integrate_position(50.0 / 1000.0);

                transforms[0].header.stamp = get_clock()->now();
                transforms[1].header.stamp = get_clock()->now();

                transforms[1].transform.rotation = tf2::toMsg(rotation);
                transforms[1].transform.translation = tf2::toMsg(position);

                tf_pub.sendTransform(transforms);

                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = get_clock()->now();
                pose.pose.orientation = tf2::toMsg(rotation);
                pose.pose.position.x = position.x();
                pose.pose.position.y = position.y();
                pose.pose.position.z = position.z();

                pose_pub->publish(pose);

                joint_states.position[0] = left_rot_amount;
                joint_states.position[1] = left_rot_amount;
                joint_states.position[2] = right_rot_amount;
                joint_states.position[3] = right_rot_amount;
                joint_states.position[4] = exc_pos;

                joint_state_pub->publish(joint_states);
            });
        }

        void integrate_position(float dt) {
            current_left_vel = limit_accel(current_left_vel, effort.left_drive, drive_accel, dt);
            current_right_vel = limit_accel(current_right_vel, effort.right_drive, drive_accel, dt);
            current_exc_vel = limit_accel(current_exc_vel, effort.lin_act, exc_accel, dt);

            float motor_to_distance_factor = 1.0 / gear_ratio / 60.0 * 2.0 * 3.14159 * wheel_radius * dt;
            position += tf2::quatRotate(rotation, tf2::Vector3(current_left_vel * motor_to_distance_factor / 2.0, 0.0, 0.0));
            position += tf2::quatRotate(rotation, tf2::Vector3(current_right_vel * motor_to_distance_factor / 2.0, 0.0, 0.0));

            tf2::Quaternion left_rot;
            left_rot.setRPY(0, 0, -current_left_vel * motor_to_distance_factor / wheelbase_width / 2.0);
            left_rot.normalize();
            rotation.normalize();
            tf2::Quaternion right_rot;
            right_rot.setRPY(0, 0, current_right_vel * motor_to_distance_factor / wheelbase_width / 2.0);
            right_rot.normalize();
            tf2::Quaternion new_rot = rotation * left_rot * right_rot;
            rotation = new_rot;
            rotation.normalize();

            left_rot_amount += current_left_vel * motor_to_distance_factor / wheel_radius;
            right_rot_amount += current_right_vel * motor_to_distance_factor / wheel_radius;
            exc_pos += current_exc_vel / 500.0 * dt;

            if (exc_pos > 0) {
                exc_pos = 0;
            } else if (exc_pos < -0.2) {
                exc_pos = -0.2;
            }
        }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<SimpleSimulator> node = std::make_shared<SimpleSimulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
