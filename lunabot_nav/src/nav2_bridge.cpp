#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lunabot_msgs/msg/event.hpp"
#include <chrono>
#include <cstdlib>
#include <rclcpp_action/client.hpp>

using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using PathMsg = nav_msgs::msg::Path;
using BoolMsg = std_msgs::msg::Bool;
using OdometryMsg = nav_msgs::msg::Odometry;
using Event = lunabot_msgs::msg::Event;
using ComputePathToPose = nav2_msgs::action::ComputePathToPose;

class Nav2Bridge : public rclcpp::Node {
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr action_compute;
  rclcpp::Subscription<PoseStampedMsg>::SharedPtr goal_sub;
  rclcpp::Subscription<PoseStampedMsg>::SharedPtr odom_sub;
  rclcpp::Publisher<PathMsg>::SharedPtr path_pub;
  rclcpp::Publisher<Event>::SharedPtr event_pub;
  rclcpp::Publisher<BoolMsg>::SharedPtr failed_pub;

  PoseStampedMsg goal;
  bool has_goal = false;
  PoseStampedMsg odom;
  bool has_odom = false;
  int plan_timeout = 0;

  bool is_planning = false;

  public:
    Nav2Bridge() : rclcpp::Node("nav2_bridge_node") {
      odom_sub = create_subscription<PoseStampedMsg>("position", 10, [this] (PoseStampedMsg value) {
          this->odom = value;
          this->has_odom = true;
      });
      goal_sub = create_subscription<PoseStampedMsg>("goal", 10, [this] (PoseStampedMsg value) {
          this->has_goal = true;
          this->goal = value;
      });
      path_pub = create_publisher<PathMsg>("nav_path", 10);
      event_pub = create_publisher<Event>("events", 10);
      failed_pub = create_publisher<BoolMsg>("nav/failed", 10);

      action_compute = rclcpp_action::create_client<ComputePathToPose>(this, "compute_path_to_pose");
      timer = create_wall_timer(std::chrono::milliseconds(500), std::bind(&Nav2Bridge::plan_path, this));
    }

  private:
    void plan_path() {
      if (plan_timeout > 3) {
        std::exit(-1);
      }
      // wait for earlier thing to finish
      if (!has_goal || !has_odom || is_planning) {
        plan_timeout++;
        return;
      }

      plan_timeout = 0;
      is_planning = true;

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
          Event event;
          event.data = Event::NO_PATH;
          event_pub->publish(event);
          BoolMsg failed;
          failed.data = true;
          failed_pub->publish(failed);
        } else {
          this->path_pub->publish(result.result->path);
          BoolMsg failed;
          failed.data = false;
          failed_pub->publish(failed);
        }
        is_planning = false;
      };

      action_compute->async_send_goal(goal, options);
    }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2Bridge>());
  rclcpp::shutdown();
  return 0;
}
