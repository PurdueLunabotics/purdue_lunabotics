#include "stheta_star.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include <memory>
#include <rcl_interfaces/msg/detail/set_parameters_result__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>

class SThetaStarPlanner : public nav2_core::GlobalPlanner {
  std::unique_ptr<SThetaStar> algorithm;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent;
  std::string name;
  rclcpp::Logger logger;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_handler;

 public:
  SThetaStarPlanner() : logger(rclcpp::get_logger("SThetaStarPlanner")) { }

  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override {
    Options options;

    this->parent = parent;
    this->name = name;

    auto node = parent.lock();
    this->logger = node->get_logger();
    nav2_util::declare_parameter_if_not_declared(node, name + ".turning_cost", rclcpp::ParameterValue(1.2));
    nav2_util::declare_parameter_if_not_declared(node, name + ".driving_cost", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, name + ".traversal_cost", rclcpp::ParameterValue(20.0));
    nav2_util::declare_parameter_if_not_declared(node, name + ".node_cost", rclcpp::ParameterValue(2.0));
    nav2_util::declare_parameter_if_not_declared(node, name + ".costmap_exponential", rclcpp::ParameterValue(2.0));
    nav2_util::declare_parameter_if_not_declared(node, name + ".max_goal_adjustment_meters", rclcpp::ParameterValue(0.75));
    nav2_util::declare_parameter_if_not_declared(node, name + ".remapping_cost_change_percent", rclcpp::ParameterValue(0.05));

    node->get_parameter(name + ".turning_cost", options.turning_cost);
    node->get_parameter(name + ".driving_cost", options.driving_cost);
    node->get_parameter(name + ".traversal_cost", options.traversal_cost);
    node->get_parameter(name + ".node_cost", options.node_cost);
    node->get_parameter(name + ".costmap_exponential", options.costmap_exponential);
    node->get_parameter(name + ".max_goal_adjustment_meters", options.max_goal_adjustment_meters);
    node->get_parameter(name + ".remapping_cost_change_percent", options.remapping_cost_change_percent);

    algorithm = std::make_unique<SThetaStar>(costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID(), options);
  }

  void cleanup() override {
    algorithm.reset();
  }

  void activate() override {
    auto node = parent.lock();
    params_handler = node->add_on_set_parameters_callback([this] (const std::vector<rclcpp::Parameter> &parameters) {
      Options options = this->algorithm->getOptions();
      rcl_interfaces::msg::SetParametersResult result;
      for (auto param : parameters) {
        if (param.get_name() == name + ".turning_cost") {
          if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            options.turning_cost = param.as_double();
          } else {
            result.reason = "Invalid type for turning cost param";
            result.successful = false;
            return result;
          }
        } else if (param.get_name() == name + ".driving_cost") {
          if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            options.driving_cost = param.as_double();
          } else {
            result.reason = "Invalid type for driving cost param";
            result.successful = false;
            return result;
          }
        } else if (param.get_name() == name + ".traversal_cost") {
          if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            options.traversal_cost = param.as_double();
          } else {
            result.reason = "Invalid type for traversal cost param";
            result.successful = false;
            return result;
          }
        } else if (param.get_name() == name + ".node_cost") {
          if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            options.node_cost = param.as_double();
          } else {
            result.reason = "Invalid type for node cost param";
            result.successful = false;
            return result;
          }
        } else if (param.get_name() == name + ".costmap_exponential") {
          if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            options.costmap_exponential = param.as_double();
          } else {
            result.reason = "Invalid type for costmap exponential param";
            result.successful = false;
            return result;
          }
        } else if (param.get_name() == name + ".max_goal_adjustment_meters") {
          if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            options.max_goal_adjustment_meters = param.as_double();
          } else {
            result.reason = "Invalid type for max goal adjustment meters";
            result.successful = false;
            return result;
          }
        } else if (param.get_name() == name + ".remapping_cost_change_percent") {
          if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            options.remapping_cost_change_percent = param.as_double();
          } else {
            result.reason = "Invalid type for remapping cost change percent";
            result.successful = false;
            return result;
          }
        }
      }
      this->algorithm->updateOptions(options);
      result.successful = true;
      return result;
    });
  }

  void deactivate() override {
    params_handler.reset();
  }

  nav_msgs::msg::Path createPlan(const PoseStampedMsg &start,
                                 const PoseStampedMsg &goal) override {
    return algorithm->createPlan(start, goal);
  }
};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(SThetaStarPlanner, nav2_core::GlobalPlanner)
