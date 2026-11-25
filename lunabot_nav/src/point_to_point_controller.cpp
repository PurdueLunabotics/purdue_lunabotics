#include <cmath>
#include <nav2_core/controller.hpp>
#include <nav2_util/node_utils.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <limits>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "pid.hpp"

namespace lunabot_nav {

enum State {
  Angular,
  Linear,
  Done
};

constexpr double pi = 3.14159265358979323846;

class PointToPointController : public nav2_core::Controller {
  rclcpp::Logger logger;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_handler;
  std::array<std::shared_ptr<rclcpp::ParameterCallbackHandle>, 3> cb_handles;

  PID linear;
  PID angular;

  double angular_inner_tolerance;
  double angular_outer_tolerance;
  double linear_tolerance;

  State state;
  int segment;
  nav_msgs::msg::Path path;
  rclcpp::Time prev_time;
  bool backwards;

  double speed_limit;
  double speed_percentage;

public:
  PointToPointController()
      : logger(rclcpp::get_logger("PointToPointController")),
        linear(1, 0, 0, 0.25),
        angular(1, 0, 0, 0.5),
        state(State::Done),
        speed_limit(0),
        speed_percentage(false) {}

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) {
    this->parent = parent;
    auto node = parent.lock();
    logger = node->get_logger();
    linear.configure(name + ".linear", node);
    angular.configure(name + ".angular", node);

    angular_inner_tolerance = nav2_util::declare_or_get_parameter(node, name + ".angular_inner_tolerance", 0.1);
    angular_outer_tolerance = nav2_util::declare_or_get_parameter(node, name + ".angular_outer_tolerance", 0.2);
    linear_tolerance = nav2_util::declare_or_get_parameter(node, name + ".linear_tolerance", 0.2);

    param_handler = std::make_shared<rclcpp::ParameterEventHandler>(node);

    cb_handles[0] = param_handler->add_parameter_callback(name + ".angular_inner_tolerance", [this](const rclcpp::Parameter &p) {
      angular_inner_tolerance = p.as_double();
    });
    cb_handles[1] = param_handler->add_parameter_callback(name + ".angular_outer_tolerance", [this](const rclcpp::Parameter &p) {
      angular_outer_tolerance = p.as_double();
    });
    cb_handles[2] = param_handler->add_parameter_callback(name + ".linear_tolerance", [this](const rclcpp::Parameter &p) {
      linear_tolerance = p.as_double();
    });
  }

  void cleanup() {
    linear = PID(1, 0, 0, 0.25);
    angular = PID(1, 0, 0, 0.5);
    reset();
  };

  void activate() {
    prev_time = parent.lock()->get_clock()->now();
  };
  void deactivate() {}

  void setPlan(const nav_msgs::msg::Path & path) {
    this->path = path;
    if (path.poses.empty())
      state = State::Done;
    else 
      state = State::Angular;
    segment = 0;
  };

  double getAngleError(const geometry_msgs::msg::PoseStamped & pose) {
    double roll, pitch, yaw;
    tf2::Matrix3x3(
        tf2::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                        pose.pose.orientation.z, pose.pose.orientation.w)
            .normalize())
        .getRPY(roll, pitch, yaw);

    double target_angle;
    if (segment < path.poses.size()) {
      assert(pose.header.frame_id == path.poses[segment].header.frame_id);
      target_angle = std::atan2(path.poses[segment].pose.position.y - pose.pose.position.y, path.poses[segment].pose.position.x - pose.pose.position.x);
    } else {
      double target_roll, target_pitch, target_yaw;
      tf2::Matrix3x3(tf2::Quaternion(path.poses[segment - 1].pose.orientation.x,
                                     path.poses[segment - 1].pose.orientation.y,
                                     path.poses[segment - 1].pose.orientation.z,
                                     path.poses[segment - 1].pose.orientation.w).normalize())
          .getRPY(target_roll, target_pitch, target_yaw);

      target_angle = target_yaw;
    }

    double angle_error = target_angle - yaw;
    
    backwards = false;

    if (angle_error > pi)
      angle_error -= 2 * pi;
    else if (angle_error < -pi)
      angle_error += 2 * pi;

    if (segment >= path.poses.size()) {
      return angle_error;
    }

    if (angle_error > pi / 2) {
      backwards = true;
      angle_error = pi - angle_error;
    } else if (angle_error < -pi / 2) {
      backwards = true;
      angle_error = -pi - angle_error;
    }

    return angle_error;
  }

  double getLinearError(const geometry_msgs::msg::PoseStamped & pose) {
    if (segment >= path.poses.size()) {
      return std::numeric_limits<float>::infinity();
    }

    return std::sqrt(
        std::pow(pose.pose.position.x - path.poses[segment].pose.position.x,
                 2) +
        std::pow(pose.pose.position.y - path.poses[segment].pose.position.y,
                 2));
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) {
    if (state == State::Done) {
      return geometry_msgs::msg::TwistStamped();
    }

    assert(pose.header.frame_id == path.header.frame_id);

    auto time = parent.lock()->get_clock()->now();

    double angular_error = getAngleError(pose);
    double linear_error = getLinearError(pose);

    while (true) {
      if (linear_error < linear_tolerance) {
        state = State::Angular;
        segment++;
        angular_error = getAngleError(pose);
        linear_error = getLinearError(pose);
      } else if (state == State::Angular && std::abs(angular_error) < angular_inner_tolerance) {
        state = segment < path.poses.size() ? State::Linear : State::Done;
      } else if (state == State::Linear && std::abs(angular_error) > angular_outer_tolerance) {
        state = State::Angular;
      } else {
        break;
      }
    }

    geometry_msgs::msg::TwistStamped output;
    output.header.frame_id = pose.header.frame_id;
    output.header.stamp = time;
    if (state == State::Angular) {
      output.twist.angular.z = angular.calculate(angular_error, (time - prev_time).seconds()) * (backwards ? -1 : 1);
    } else if (state == State::Linear) {
      output.twist.linear.x = linear.calculate(linear_error, (time - prev_time).seconds(), speed_limit, speed_percentage) * (backwards ? -1 : 1);
    }
    prev_time = time;

    return output;
  };

  bool cancel() {
    reset();
    return true;
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) {
    this->speed_limit = speed_limit;
    this->speed_percentage = percentage;
  }

  void reset() {
    setPlan(nav_msgs::msg::Path());
  }
};

};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(lunabot_nav::PointToPointController, nav2_core::Controller)
