#include <lunabot_control/mpc.h>
#include <math.h>

MPC::MPC(ros::NodeHandle *nh) {
  double frequency;
  std::string map_topic, path_topic, odom_topic, cmd_vel_topic;
  // Global params
  ros::param::get("/odom_topic", odom_topic);
  ros::param::get("/cmd_vel_topic", cmd_vel_topic);

  // Nav params
  ros::param::get("map_topic", map_topic);
  ros::param::get("global_path_topic", path_topic);

  // MPC params
  ros::param::get("~rollout_count", this->rollout_count_);
  ros::param::get("~top_rollouts", this->top_rollouts_);
  ros::param::get("~iterations", this->iterations_);
  ros::param::get("~w_linear", this->w_linear_);
  ros::param::get("~w_angular", this->w_angular_);
  ros::param::get("~w_waypoint_pos", this->w_waypoint_pos_);
  ros::param::get("~w_accel", this->w_accel_);
  ros::param::get("~w_waypoint_rot", this->w_waypoint_rot_);
  ros::param::get("~w_occupied", this->w_occupied_);
  ros::param::get("~horizon_length", this->horizon_length_);
  ros::param::get("~frequency", frequency);
  ros::param::get("~min_distance_threshold", this->min_dist_thres_);
  ros::param::get("~velocity_limits/linear", lin_lim_);
  ros::param::get("~velocity_limits/angular", ang_lim_);

  // MPC Variables
  this->delta_time_ = 1 / frequency;
  this->path_ind_ = 0;
  this->enabled_ = false;

  // Ros publishers and subscribers
  this->velocity_pub_ = nh->advertise<geometry_msgs::Twist>(cmd_vel_topic, 10);
  this->grid_sub_ = nh->subscribe(map_topic, 10, &MPC::update_grid, this);
  this->path_sub_ = nh->subscribe(path_topic, 10, &MPC::update_path,
                                  this); // Convert to Map
  this->robot_pos_sub_ = nh->subscribe(odom_topic, 10, &MPC::update_robot_pos,
                                       this); // Convert to Map
}

bool MPC::check_collision_(Eigen::RowVectorXd pos) {
  std::vector<double> state = {pos(0), pos(1)};

  return this->map_.occupied_at_pos(state);
}

int MPC::is_close_() { return dist_to_setpoint_() <= min_dist_thres_ * min_dist_thres_; }

void MPC::update_setpoint_() {
  if (is_close_()) {
    if (this->path_ind_ == this->path_.size() - 1) {
      publish_velocity_(0, 0);
      this->path_ind_ = 0;
      this->enabled_ = false;
    } else {
      this->path_ind_++;
    }
  }
}

void MPC::update_grid(const nav_msgs::OccupancyGrid &grid) { map_.update_map(grid); }

void MPC::update_path(const nav_msgs::Path &path) {
  ROS_INFO("PATH");
  this->path_.clear();
  this->path_ind_ = 0;
  for (int i = 0; i < path.poses.size(); ++i) {
    geometry_msgs::Point position = path.poses[i].pose.position;
    std::vector<double> pos;
    pos.push_back(position.x);
    pos.push_back(position.y);
    pos.push_back(get_yaw_(path.poses[i].pose.orientation));
    this->path_.push_back(pos);
  }

  this->enabled_ = true;
}

double MPC::get_yaw_(geometry_msgs::Quaternion q) {
  return std::atan2(2 * (q.z * q.w + q.x * q.y), 1 - 2 * (q.z * q.z + q.y * q.y));
}

void MPC::update_robot_pos(const nav_msgs::Odometry &odometry) {
  geometry_msgs::Pose pose = odometry.pose.pose;
  this->robot_pos_.clear();
  this->robot_pos_.push_back(pose.position.x);
  this->robot_pos_.push_back(pose.position.y);
  this->robot_pos_.push_back(get_yaw_(pose.orientation));
}

void MPC::publish_velocity_(double linear, double angular) {
  geometry_msgs::Twist twist;
  twist.linear.x = linear;
  twist.angular.z = angular;
  this->velocity_pub_.publish(twist);
}

// See Boundary and Constraint Handling
// https://cma-es.github.io/cmaes_sourcecode_page.html#practical
double MPC::smooth_clamp_(double x, double a, double b) {
  return a + (b - a) * (1 + sin(M_PI * x / 2)) / 2;
}

std::pair<std::vector<Eigen::MatrixXd>, std::vector<Eigen::MatrixXd>>
MPC::normal_distribute_(Eigen::MatrixXd means, Eigen::MatrixXd std_devs, int count) {
  std::vector<Eigen::MatrixXd> random_vels_raw(count);
  std::vector<Eigen::MatrixXd> random_vels_clamped(count);

#pragma omp parallel for
  for (int i = 0; i < count; ++i) {
    Eigen::MatrixXd random_vel(means.rows(), means.cols());
    Eigen::MatrixXd random_vel_clamped(means.rows(), means.cols());

#pragma omp parallel for
    for (int j = 0; j < means.rows(); ++j) {

#pragma omp parallel for
      for (int k = 0; k < means.cols(); ++k) {
        std::random_device rd;
        std::default_random_engine generator(rd());
        std::normal_distribution<double> distribution(means(j, k), std_devs(j, k));
        double rand_vel = distribution(generator);
        double rand_vel_clamped = 0;
        if (k == 0) {
          rand_vel_clamped = smooth_clamp_(rand_vel, lin_lim_[0], lin_lim_[1]);
        } else {
          rand_vel_clamped = smooth_clamp_(rand_vel, ang_lim_[0], ang_lim_[1]);
        }
        random_vel(j, k) = rand_vel;
        random_vel_clamped(j, k) = rand_vel_clamped;
      }
    }
    random_vels_raw[i] = random_vel;
    random_vels_clamped[i] = random_vel_clamped;
  }
  return std::make_pair(random_vels_clamped, random_vels_raw);
}

Eigen::MatrixXd MPC::mean_(std::vector<std::pair<Eigen::MatrixXd, double>> rollouts) {
  Eigen::MatrixXd mean = Eigen::MatrixXd::Zero(this->horizon_length_, 2);
#pragma omp parallel for
  for (int i = 0; i < this->top_rollouts_; ++i) {
#pragma omp parallel for
    for (int j = 0; j < rollouts[i].first.rows(); ++j) {
      mean(j, 0) += rollouts[i].first(j, 5);
      mean(j, 1) += rollouts[i].first(j, 6);
    }
  }

  return mean / this->top_rollouts_;
}

Eigen::MatrixXd MPC::std_dev_(std::vector<std::pair<Eigen::MatrixXd, double>> rollouts) {
  Eigen::MatrixXd temp_mean = mean_(rollouts);
  Eigen::MatrixXd std_dev = Eigen::MatrixXd::Zero(this->horizon_length_, 2);

#pragma omp parallel for
  for (int i = 0; i < this->top_rollouts_; ++i) {
#pragma omp parallel for
    for (int j = 0; j < rollouts[i].first.rows(); ++j) {
      std_dev(j, 0) +=
          (rollouts[i].first(j, 5) - temp_mean(j, 0)) * (rollouts[i].first(j, 5) - temp_mean(j, 0));
      std_dev(j, 1) +=
          (rollouts[i].first(j, 6) - temp_mean(j, 1)) * (rollouts[i].first(j, 6) - temp_mean(j, 1));
    }
  }
  ROS_ASSERT(this->top_rollouts_ > 1);

  std_dev /= this->top_rollouts_ - 1;
#pragma omp parallel for
  for (int i = 0; i < std_dev.rows(); ++i) {
#pragma omp parallel for
    for (int j = 0; j < std_dev.cols(); ++j) {
      std_dev(i, j) = std::sqrt(std_dev(i, j));
    }
  }
  return std_dev;
}

double MPC::dist_to_setpoint_() {
  return ((robot_pos_[0] - path_[path_ind_][0]) * (robot_pos_[0] - path_[path_ind_][0]) +
          (robot_pos_[1] - path_[path_ind_][1]) * (robot_pos_[1] - path_[path_ind_][1]));
}

double MPC::calculate_cost_(Eigen::MatrixXd rollout) {
  SWRI_PROFILE("calculate-cost");
  double cost = 0;
  std::vector<double> robot_pos = this->robot_pos_;
  std::vector<double> goal = this->goal_;
#pragma omp parallel for reduction(+ : cost)
  for (int i = 0; i < rollout.rows(); ++i) {
    Eigen::RowVectorXd position = rollout.row(i);
    cost += this->w_linear_ * ((position(0) - robot_pos[0]) * (position(0) - robot_pos[0]) +
                               (position(1) - robot_pos[1]) * (position(1) - robot_pos[1]));
    cost += this->w_angular_ * (position(2) - robot_pos[2]) * (position(2) - robot_pos[2]);
    int path_cost_i = std::min(path_ind_ + i, (int)path_.size() - 1);
    cost += this->w_waypoint_pos_ *
            ((position(0) - path_[path_cost_i][0]) * (position(0) - path_[path_cost_i][0]) +
             (position(1) - path_[path_cost_i][1]) * (position(1) - path_[path_cost_i][1]));

    cost += this->w_waypoint_rot_ * (position(2) - path_[path_cost_i][2]) *
            (position(2) - path_[path_cost_i][2]);
    cost += this->w_occupied_ * check_collision_(position);
    if (i > 0) {
      cost += this->w_accel_ * rollout(i, 3) - rollout(i - 1, 3); // min accel
      cost += this->w_accel_ * rollout(i, 4) - rollout(i - 1, 4); // min accel
    }
  }

  return cost;
}

Eigen::MatrixXd MPC::calculate_model_(Eigen::MatrixXd actions, Eigen::MatrixXd actions_raw) {
  SWRI_PROFILE("calculate-model");
  Eigen::VectorXd current_pos(3); // 3 x 1 mat
  current_pos << this->robot_pos_[0], this->robot_pos_[1], this->robot_pos_[2];
  Eigen::MatrixXd model(this->horizon_length_, 7); // x, y, theta, v, omega, v_raw, omega_raw

  for (int i = 0; i < this->horizon_length_; ++i) {
    model(i, 0) = current_pos(0);
    model(i, 1) = current_pos(1);
    model(i, 2) = current_pos(2);
    model(i, 3) = actions(i, 0);
    model(i, 4) = actions(i, 1);
    model(i, 5) = actions_raw(i, 0);
    model(i, 6) = actions_raw(i, 1);
    current_pos(0) += std::cos(current_pos(2)) * this->delta_time_ * actions(i, 0);
    current_pos(1) += std::sin(current_pos(2)) * this->delta_time_ * actions(i, 0);
    current_pos(2) += this->delta_time_ * actions(i, 1);
  }

  return model;
}

bool comparator(std::pair<Eigen::MatrixXd, double> a, std::pair<Eigen::MatrixXd, double> b) {
  return a.second < b.second;
}

void MPC::calculate_velocity() {
  SWRI_PROFILE("calculate-velocity");
  if (this->robot_pos_.empty() || this->path_.empty() || !this->enabled_) {
    return;
  }
  int horizon_length = this->horizon_length_;
  Eigen::MatrixXd means = Eigen::MatrixXd::Zero(horizon_length, 2);    // v, omega
  Eigen::MatrixXd std_devs = Eigen::MatrixXd::Ones(horizon_length, 2); // v, omega
  int iterations = this->iterations_;
  int rollout_count = this->rollout_count_;
  int top_rollouts = this->top_rollouts_;
#pragma omp parallel for
  for (int i = 0; i < iterations; ++i) {
    std::pair<std::vector<Eigen::MatrixXd>, std::vector<Eigen::MatrixXd>> random_actions =
        normal_distribute_(means, std_devs, rollout_count_);
    std::vector<std::pair<Eigen::MatrixXd, double>> rollouts(
        rollout_count_); // x, y, theta, v, omega for each horizon step
                         // paired with cost

#pragma omp parallel for
    for (int j = 0; j < rollout_count_; ++j) {
      Eigen::MatrixXd state = calculate_model_(random_actions.first[j], random_actions.second[j]);
      rollouts[j] = std::make_pair(state, calculate_cost_(state));
    }

    // Getting best options
    std::sort(rollouts.begin(), rollouts.end(), comparator);
    if (i == iterations - 1) {
      publish_velocity_(rollouts[0].first.coeff(0, 3), rollouts[0].first.coeff(0, 4));
    } else {
      means = mean_(rollouts);
      std_devs = std_dev_(rollouts);
    }
  }
  this->update_setpoint_();

  std::cout << "curr z_rot: " << robot_pos_[2] << std::endl;
  std::cout << "waypoint z_rot: " << path_[path_ind_][2] << std::endl;
}
