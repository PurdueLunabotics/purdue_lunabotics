#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using PoseStampedMsg = geometry_msgs::msg::PoseStamped;

class GoalNav2Bridge: public rclcpp::Node {
  rclcpp::Subscription<PoseStampedMsg>::SharedPtr goal_sub;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr actionClient;

  bool ready = false;

  public:
    GoalNav2Bridge(): rclcpp::Node("nav2_bridge_node") {
      goal_sub = create_subscription<PoseStampedMsg>("goal", 10, [this](PoseStampedMsg goal) {this->goal_callback(goal);});
      actionClient = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");

      bool gotActionServer = actionClient->wait_for_action_server(std::chrono::seconds{30});
      if (!gotActionServer) {
        RCLCPP_ERROR(this->get_logger(), "Goal->nav2 bridge couldn't get to action server! Quitting.");
        return;
      }

      ready = true;
      RCLCPP_INFO(this->get_logger(), "Goal->Nav2 bridge is running successfully.");
    }

  private:
    void goal_callback(PoseStampedMsg goal) {
      if (!ready) {
        RCLCPP_INFO(this->get_logger(), "Goal->nav2 bridge ignoring goal, not connected to action server yet.");
      }

      nav2_msgs::action::NavigateToPose::Goal navGoal;
      navGoal.pose = goal;
      navGoal.pose.header.stamp = this->now();

      actionClient->async_send_goal(navGoal);
    }

};

int main() {
  rclcpp::init(0, nullptr);
  rclcpp::spin(std::make_shared<GoalNav2Bridge>());
  rclcpp::shutdown();
  return 0;
}
