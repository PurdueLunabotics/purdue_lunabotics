#ifndef MPC_H
#define MPC_H

#include <algorithm>
#include <assert.h>
#include <bits/stdc++.h>
#include <cmath>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>

#include <Eigen/Dense>

#include <lunabot_control/map.h>

using std::placeholders::_1;

class MPC : public rclcpp::Node {
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::Subscription < nav_msgs::msg::OccupancyGrid >::SharedPtr grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pos_sub_;

  int rollout_count_;
  int top_rollouts_;
  int iterations_;
  double w_linear_;
  double w_angular_;
  double w_waypoint_;
  double w_occupied_;
  double min_dist_thres_;
  int path_ind_;
  bool enabled_;
  std::vector<std::vector<double>> path_;
  std::vector<double> goal_;
  std::vector<double> ang_lim_;
  std::vector<double> lin_lim_;
  std::vector<double> robot_pos_;
  int horizon_length_;
  double delta_time_;
  Map map_;

  double smooth_clamp_(double x, double a, double b);
  bool check_collision_(Eigen::RowVectorXd pos);
  double find_closest_distance_(Eigen::RowVectorXd pos);
  double calculate_cost_(Eigen::MatrixXd rollout);
  Eigen::MatrixXd calculate_model_(Eigen::MatrixXd actions);
  std::vector<Eigen::MatrixXd> normal_distribute_(Eigen::MatrixXd means, Eigen::MatrixXd std_devs,
                                                  int count);
  Eigen::MatrixXd mean_(std::vector<std::pair<Eigen::MatrixXd, double>>);
  Eigen::MatrixXd std_dev_(std::vector<std::pair<Eigen::MatrixXd, double>>);
  double clamp_(double val, double low, double high);

  bool is_close_();
  void update_setpoint_();
  double dist_to_setpoint_();

public:
  MPC();
  void update_grid(const nav_msgs::msg::OccupancyGrid &grid);
  void update_path(const nav_msgs::msg::Path &path);
  void update_robot_pos(const nav_msgs::msg::Odometry &odometry);
  void calculate_velocity();
  void publish_velocity(double linear, double angular);

};

#endif
