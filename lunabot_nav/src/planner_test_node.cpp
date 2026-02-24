#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include <chrono>
#include <memory>
#include <rclcpp/time.hpp>
#include <rclcpp_action/client.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

using OccupancyGridMsg = nav_msgs::msg::OccupancyGrid;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using PoseStampedWithCovMsg = geometry_msgs::msg::PoseWithCovarianceStamped;
using PathMsg = nav_msgs::msg::Path;
using ComputePathToPose = nav2_msgs::action::ComputePathToPose;

class PlannerTestNode : public rclcpp::Node {
  rclcpp_action::Client<ComputePathToPose>::SharedPtr action_compute;
  rclcpp::Subscription<PoseStampedMsg>::SharedPtr goal_sub;
  rclcpp::Subscription<PoseStampedWithCovMsg>::SharedPtr odom_sub;
  rclcpp::Publisher<PathMsg>::SharedPtr path_pub;
  rclcpp::TimerBase::SharedPtr timer;

  PoseStampedMsg goal;
  PoseStampedMsg odom;

  public:
    PlannerTestNode() : rclcpp::Node("planner_test_node") {
      path_pub = create_publisher<PathMsg>("nav_path", 10);
      odom_sub = create_subscription<PoseStampedWithCovMsg>("initialpose", 10, [this] (PoseStampedWithCovMsg value) {
          this->odom.header = value.header;
          this->odom.pose = value.pose.pose;
          plan_path();
      });
      goal_sub = create_subscription<PoseStampedMsg>("goal", 10, [this] (PoseStampedMsg value) {
          this->goal = value;
          plan_path();
      });
      action_compute = rclcpp_action::create_client<ComputePathToPose>(this, "compute_path_to_pose");
      timer = create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerTestNode::plan_path, this));
    }

    void plan_path() {
      if (!action_compute->wait_for_action_server()) {
        RCLCPP_WARN(get_logger(), "Action server not ready yet");
        return;
      }

      auto goal = ComputePathToPose::Goal();
      goal.goal = this->goal;
      goal.start = odom; 
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

      action_compute->async_send_goal(goal, options);
    }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerTestNode>());
  rclcpp::shutdown();
  return 0;
}
