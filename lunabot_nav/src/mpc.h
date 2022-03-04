#ifndef MPC_H
#define MPC_H

#include <ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>

#include <bits/stdc++.h>
#include <Eigen/Dense>
#include <random>
#include <vector>

class MPC {
private:
    ros::NodeHandle nh;
    ros::Publisher velocity_pub;
    ros::Subscriber grid_sub;
    ros::Subscriber path_sub;
    ros::Subscriber goal_sub;
    ros::Subscriber robot_pos_sub;

    int rollout_count;
    int top_rollouts;
    int iterations;
    double w_linear;
    double w_angular;
    double w_goal;
    double w_line;
    double w_occupied;
    std::vector<std::vector<double>> path;
    std::vector<double> goal;
    std::vector<double> robot_pos;
    int horizon_length;
    double delta_time;
    std::vector<std::vector<int>> grid;
    bool check_collision(Eigen::RowVectorXd pos);
    double find_closest_distance(Eigen::RowVectorXd pos);
    double calculate_cost(Eigen::MatrixXd rollout);
    Eigen::MatrixXd calculate_model(Eigen::MatrixXd actions);
    std::vector<Eigen::MatrixXd> normal_distribute(Eigen::MatrixXd means, Eigen::MatrixXd std_devs, int count);
    void publish_velocity(double linear, double angular);
    Eigen::MatrixXd mean(std::vector<std::pair<Eigen::MatrixXd, double>>);
    Eigen::MatrixXd std_dev(std::vector<std::pair<Eigen::MatrixXd, double>>);

public:
    MPC(int rollout_count, int top_rollouts, int iterations, double w_linear, double w_angular, double w_goal, double w_line, double w_occupied, int horizon_length, double delta_time);
    void update_grid(nav_msgs::OccupancyGrid grid);
    void update_path(nav_msgs::Path path);
    void update_goal(geometry_msgs::PoseStamped pose);
    void update_robot_pos(nav_msgs::Odometry odometry);
    void calculate_velocity();
};

#endif MPC_H