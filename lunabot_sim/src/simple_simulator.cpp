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

        float wheel_radius = 0.2;
        float wheelbase_width = 0.32; // half
        float wheelbase_length = 0.3; // half
        float gear_ratio = 50.0;
        float turn_slip = 0.5;

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
            float motor_to_distance_factor = 1.0 / gear_ratio / 60.0 * 2.0 * 3.14159 * wheel_radius * dt;
            position += tf2::quatRotate(rotation, tf2::Vector3((float) effort.left_drive * motor_to_distance_factor / 2.0, 0.0, 0.0));
            position += tf2::quatRotate(rotation, tf2::Vector3((float) effort.right_drive * motor_to_distance_factor / 2.0, 0.0, 0.0));

            tf2::Quaternion left_rot;
            left_rot.setRPY(0, 0, (float) -effort.left_drive * motor_to_distance_factor / wheelbase_width / 2.0 * turn_slip);
            left_rot.normalize();
            rotation.normalize();
            tf2::Quaternion right_rot;
            right_rot.setRPY(0, 0, (float) effort.right_drive * motor_to_distance_factor / wheelbase_width / 2.0 * turn_slip);
            right_rot.normalize();
            tf2::Quaternion new_rot = rotation * left_rot * right_rot;
            rotation = new_rot;
            rotation.normalize();

            left_rot_amount += (float) effort.left_drive * motor_to_distance_factor / wheel_radius;
            right_rot_amount += (float) effort.right_drive * motor_to_distance_factor / wheel_radius;
            exc_pos += (float) effort.lin_act / 500.0 * dt;

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
