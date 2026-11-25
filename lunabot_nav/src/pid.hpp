#pragma once

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <memory>

namespace lunabot_nav {

class PID {
  double p;
  double i;
  double d;
  double max_output;

  double prev_error;
  double sum;


  std::shared_ptr<rclcpp::ParameterEventHandler> param_handler;
  std::array<std::shared_ptr<rclcpp::ParameterCallbackHandle>, 4> cb_handles;

public:
  PID(double p, double i, double d, double max_output);

  void configure(std::string name, std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
  void cleanup();

  double calculate(double error, double dt, double speed_limit = 0.0, bool percent = false);
  void reset();
};

};
