#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <rclcpp_action/client.hpp>

using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using PathMsg = nav_msgs::msg::Path;
using OdometryMsg = nav_msgs::msg::Odometry;
using ComputePathToPose = nav2_msgs::action::ComputePathToPose;

class Nav2Bridge : public rclcpp::Node {
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr action;
  rclcpp::Subscription<PoseStampedMsg>::SharedPtr goal_sub;
  rclcpp::Subscription<OdometryMsg>::SharedPtr odom_sub;
  rclcpp::Publisher<PathMsg>::SharedPtr path_pub;

  PoseStampedMsg goal;
  OdometryMsg odom;

  public:
    Nav2Bridge() : rclcpp::Node("nav2_bridge_node") {
      odom_sub = create_subscription<OdometryMsg>("/rtabmap/odom", 10, [this] (OdometryMsg value) {
          this->odom = value;
      });
      goal_sub = create_subscription<PoseStampedMsg>("/goal", 10, [this] (PoseStampedMsg value) {
          this->goal = value;
      });
      path_pub = create_publisher<PathMsg>("/test_path", 10);

      action = rclcpp_action::create_client<ComputePathToPose>(this, "/compute_path_to_pose");
      timer = create_wall_timer(std::chrono::milliseconds(500), std::bind(&Nav2Bridge::plan_path, this));
    }

  private:
    void plan_path() {
      if (!action->wait_for_action_server()) {
        RCLCPP_WARN(get_logger(), "Action server not ready yet");
        return;
      }

      auto goal = ComputePathToPose::Goal();
      goal.goal = this->goal;
      goal.start.pose = odom.pose.pose; 
      goal.start.header = odom.header; 
      goal.use_start = true;
      goal.planner_id = "GridBased";

      auto options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
      options.result_callback = [this] (rclcpp_action::ClientGoalHandle<ComputePathToPose>::WrappedResult result) {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_WARN(get_logger(), "Failed to compute pose");
        } else {
          this->path_pub->publish(result.result->path);
        }
      };

      action->async_send_goal(goal, options);
    }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2Bridge>());
  rclcpp::shutdown();
  return 0;
}
