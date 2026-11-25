#include "pid.hpp"
#include <memory>
#include <nav2_util/node_utils.hpp>

using namespace lunabot_nav;

PID::PID(double p, double i, double d, double max_output) : p(p), i(i), d(d), max_output(max_output), logger(rclcpp::get_logger("PID")) {
  reset();
}

void PID::configure(std::string name, std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) {
  p = nav2_util::declare_or_get_parameter(node, name + ".p", p);
  i = nav2_util::declare_or_get_parameter(node, name + ".i", i);
  d = nav2_util::declare_or_get_parameter(node, name + ".d", d);
  max_output = nav2_util::declare_or_get_parameter(node, name + ".max_output", max_output);

  param_handler = std::make_shared<rclcpp::ParameterEventHandler>(node);

  logger = node->get_logger().get_child(name);

  cb_handles[0] = param_handler->add_parameter_callback(name + ".p", [this](const rclcpp::Parameter &p) {
    this->p = p.as_double();
    reset();
  });
  cb_handles[1] = param_handler->add_parameter_callback(name + ".i", [this](const rclcpp::Parameter &p) {
    this->i = p.as_double();
    reset();
  });
  cb_handles[2] = param_handler->add_parameter_callback(name + ".d", [this](const rclcpp::Parameter &p) {
    this->d = p.as_double();
    reset();
  });
  cb_handles[3] = param_handler->add_parameter_callback(name + ".max_output", [this](const rclcpp::Parameter &p) {
    this->max_output = p.as_double();
    reset();
  });
}

double PID::calculate(double error, double dt, double speed_limit, bool percentage) {
  sum += error * dt;
  double velocity = (error - prev_error) / dt;
  prev_error = error;

  double output = p * error + d * velocity + i * sum;

  double current_max_output = max_output;
  if (speed_limit != 0.0 && percentage) {
    current_max_output *= speed_limit / 100.0;
  } else if (speed_limit != 0.0) {
    current_max_output = speed_limit;
  }

  if (output > current_max_output) {
    return current_max_output;
  } else if (output < -current_max_output) {
    return -current_max_output;
  } else {
    return output;
  }
}

void PID::reset() {
  sum = 0;
  prev_error = 0;
}
