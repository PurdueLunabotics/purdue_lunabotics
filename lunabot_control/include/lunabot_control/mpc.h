#ifndef MPC_H
#define MPC_H

#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <swri_profiler/profiler.h>

#include <Eigen/Dense>
#include <assert.h>
#include <bits/stdc++.h>
#include <cmath>
#include <random>
#include <vector>

#include <lunabot_control/map.h>

class MPC {
private:
  ros::Publisher velocity_pub_;
  ros::Subscriber grid_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber robot_pos_sub_;

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

  int is_close_();
  void update_setpoint_();
  double dist_to_setpoint_();

public:
  MPC(ros::NodeHandle *nh);
  void update_grid(const nav_msgs::OccupancyGrid &grid);
  void update_path(const nav_msgs::Path &path);
  void update_robot_pos(const nav_msgs::Odometry &odometry);
  void calculate_velocity();
  void publish_velocity_(double linear, double angular);

};

#endif
