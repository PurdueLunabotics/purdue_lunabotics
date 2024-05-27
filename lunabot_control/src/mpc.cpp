#include <lunabot_control/mpc.h>

MPC::MPC() : Node("mpc_node") {
  double frequency;
  std::string map_topic, path_topic, odom_topic, cmd_vel_topic;
  // Global params
  ros::param::get("/odom_topic", odom_topic);
  ros::param::get("/cmd_vel_topic", cmd_vel_topic);

  // Nav params
  ros::param::get("map_topic", map_topic);
  ros::param::get("global_path_topic", path_topic);

  // MPC params
  ros::param::get("~rollout_count", this->rollout_count_); // Number of samples to generate
  ros::param::get("~top_rollouts", this->top_rollouts_); // The top n samples to use for the next iteration
  ros::param::get("~iterations", this->iterations_);
  ros::param::get("~w_linear", this->w_linear_); // weights for the cost function
  ros::param::get("~w_angular", this->w_angular_);
  ros::param::get("~w_waypoint", this->w_waypoint_);
  ros::param::get("~w_occupied", this->w_occupied_);
  ros::param::get("~horizon_length", this->horizon_length_); // How many steps in the future to look ahead
  ros::param::get("~frequency", frequency);
  ros::param::get("~min_distance_threshold", this->min_dist_thres_); // How close the robot must be to a targeted point
  ros::param::get("~velocity_limits/linear", lin_lim_);
  ros::param::get("~velocity_limits/angular", ang_lim_);

  // MPC Variables
  this->delta_time_ = 1 / frequency;
  this->path_ind_ = 0; // Keep track of where on the path we're targeting
  this->enabled_ = false;

  // Ros publishers and subscribers
  this->velocity_pub_ = nh->advertise<geometry_msgs::Twist>(cmd_vel_topic, 10);
  this->grid_sub_ = nh->subscribe(map_topic, 10, &MPC::update_grid, this);
  this->path_sub_ = nh->subscribe(path_topic, 10, &MPC::update_path, this); // Convert to Map
  this->robot_pos_sub_ = nh->subscribe(odom_topic, 10, &MPC::update_robot_pos, this); // Convert to Map
}

// The columns of a generated sample
enum SampleColumns {
  SAMPLE_LINEAR_VEL = 0,
  SAMPLE_ANGULAR_VEL = 1,
};

// The columns of a modeled sample
enum ModelColumns {
  X_POS = 0,
  Y_POS = 1,
  YAW = 2,
  MODEL_LINEAR_VEL = 3,
  MODEL_ANGULAR_VEL = 4,
};

// Check if a given position (2d coord in real space) is occupied
bool MPC::check_collision_(Eigen::RowVectorXd pos) {
  std::vector<double> state = {pos(0), pos(1)};

  return this->map_.occupied_at_pos(state);
}

double MPC::find_closest_distance_(Eigen::RowVectorXd pos) {
  std::vector<std::vector<double>> path = this->path_;

  if (path.empty()) {
    return -1;
  }

  double min_dist = -1;

  std::vector<double> point = path[0];
  for (int i = 1; i < path.size(); ++i) {
    std::vector<double> new_point = path[i];

    // Building Parametric Lines
    double x_vel = new_point[0] - point[0];
    double y_vel = new_point[1] - point[1];

    if (x_vel == 0 && y_vel == 0) {
      return 0;
    }

    double t = (x_vel * (pos(0) - point[0]) + y_vel * (pos(1) - point[1])) /
               (x_vel * x_vel + y_vel * y_vel); // Finding time where line is closest to point

    // Scaling to be on the line segment
    if (t < 0) {
      t = 0;
    } else if (t > 1) {
      t = 1;
    }

    double x_closest = point[0] + t * x_vel;
    double y_closest = point[1] + t * y_vel;

    double distance = std::sqrt((x_closest - pos(0)) * (x_closest - pos(0)) +
                                (y_closest - pos[1]) * (y_closest - pos(1)));

    if (min_dist == -1 || distance < min_dist) {
      min_dist = distance;
    }

    point = new_point;
  }
  return min_dist;
}

// Check if the robot within the distance threshold of the target point on the path (squared, for faster computation)
bool MPC::is_close_() { 
  return dist_to_setpoint_() <= min_dist_thres_ * min_dist_thres_; 
}

// Check if the robot has reached the target point- if so, move further down the path.
// If reached the end of the path, stop the robot.
void MPC::update_setpoint_() {
  if (is_close_()) {
    if (this->path_ind_ == this->path_.size() - 1) {
      publish_velocity(0, 0);
      this->path_ind_ = 0;
      this->enabled_ = false;
    } else {
      this->path_ind_++;
    }
  }
}

void MPC::update_grid(const nav_msgs::OccupancyGrid &grid) { 
  map_.update_map(grid); 
}

// Create a new path when given a new path. Reset the target point to the first point on the path.
void MPC::update_path(const nav_msgs::Path &path) {
  ROS_DEBUG("MPC: New path");
  this->path_.clear();
  this->path_ind_ = 0;
  for (int i = 0; i < path.poses.size(); ++i) {
    geometry_msgs::Point position = path.poses[i].pose.position;
    std::vector<double> pos;
    pos.push_back(position.x);
    pos.push_back(position.y);
    this->path_.push_back(pos);
  }

  this->enabled_ = true; // Re-enable path following
}

static double get_yaw_(geometry_msgs::Quaternion q) {
  return std::atan2(2 * (q.z * q.w + q.x * q.y), 1 - 2 * (q.z * q.z + q.y * q.y));
}

void MPC::update_robot_pos(const nav_msgs::Odometry &odometry) {
  geometry_msgs::Pose pose = odometry.pose.pose;
  this->robot_pos_.clear();
  this->robot_pos_.push_back(pose.position.x);
  this->robot_pos_.push_back(pose.position.y);
  this->robot_pos_.push_back(get_yaw_(pose.orientation));
}

double MPC::clamp_(double val, double low, double high) {
  if (val < low) {
    return low;
  } else if (val > high) {
    return high;
  } else {
    return val;
  }
}

// Clamp the given velocity with the velocity limits, and publish the new cmd_vel
void MPC::publish_velocity(double linear, double angular) {
  geometry_msgs::Twist twist;
  linear = clamp_(linear, lin_lim_[0], lin_lim_[1]);
  angular = clamp_(angular, ang_lim_[0], ang_lim_[1]);
  twist.linear.x = linear;
  twist.angular.z = angular;
  this->velocity_pub_.publish(twist);
}

// See Boundary and Constraint Handling
// https://cma-es.github.io/cmaes_sourcecode_page.html#practical
double MPC::smooth_clamp_(double x, double a, double b) {
  return a + (b - a) * (1 + sin(M_PI * x / 2)) / 2;
}

// Generate random samples (velocities) based on the current means and standard deviations
// The means, std_devs, and samples are of shape (horizon_length, 2) where the columns are (linear, angular)
std::vector<Eigen::MatrixXd> MPC::normal_distribute_(Eigen::MatrixXd means, Eigen::MatrixXd std_devs, int count) {
  std::vector<Eigen::MatrixXd> random_vels(count);

  for (int i = 0; i < count; ++i) {
    Eigen::MatrixXd random_vel(means.rows(), means.cols());

    // For each sample, go through each horizon step and lin/ang velocity
    for (int j = 0; j < means.rows(); ++j) {
      for (int k = 0; k < means.cols(); ++k) {

        // Generate a normal distribution based on the mean and std_dev
        std::random_device rd;
        std::default_random_engine generator(rd());
        std::normal_distribution<double> distribution(means(j, k), std_devs(j, k));

        // Pick a random velocity from the distribution
        double rand_vel = distribution(generator);
        random_vel(j, k) = rand_vel;
      }
    }
    // Add it to the vector of samples
    random_vels[i] = random_vel;
  }
  return random_vels;
}

// Calculate the new means based on the best samples. Input is the sorted, weighted list of modeled samples.
// Finds the mean for each horizon step and lin/ang velocity (the shape of a sample)
Eigen::MatrixXd MPC::mean_(std::vector<std::pair<Eigen::MatrixXd, double>> rollouts) {
  Eigen::MatrixXd mean = Eigen::MatrixXd::Zero(this->horizon_length_, 2);
  for (int i = 0; i < this->top_rollouts_; ++i) {
    for (int j = 0; j < rollouts[i].first.rows(); ++j) {
      mean(j, SAMPLE_LINEAR_VEL) += rollouts[i].first(j, MODEL_LINEAR_VEL);
      mean(j, SAMPLE_ANGULAR_VEL) += rollouts[i].first(j, MODEL_ANGULAR_VEL);
    }
  }
  return mean / this->top_rollouts_;
}


// Calculates the new standard deviations similarly (input is the sorted, weighted list of modeled samples),
// Reurns the std_dev for each horizon step and lin/ang velocity (the shape of a sample)
Eigen::MatrixXd MPC::std_dev_(std::vector<std::pair<Eigen::MatrixXd, double>> rollouts) {
  Eigen::MatrixXd temp_mean = mean_(rollouts);
  Eigen::MatrixXd std_dev = Eigen::MatrixXd::Zero(this->horizon_length_, 2);

  for (int i = 0; i < this->top_rollouts_; ++i) {
    for (int j = 0; j < rollouts[i].first.rows(); ++j) {
      std_dev(j, SAMPLE_LINEAR_VEL) += (rollouts[i].first(j, MODEL_LINEAR_VEL) - temp_mean(j, SAMPLE_LINEAR_VEL)) * 
                                       (rollouts[i].first(j, MODEL_LINEAR_VEL) - temp_mean(j, SAMPLE_LINEAR_VEL));
      std_dev(j, SAMPLE_ANGULAR_VEL) += (rollouts[i].first(j, MODEL_ANGULAR_VEL) - temp_mean(j, SAMPLE_ANGULAR_VEL)) * 
                                        (rollouts[i].first(j, MODEL_ANGULAR_VEL) - temp_mean(j, SAMPLE_ANGULAR_VEL));
    }
  }
  ROS_ASSERT(this->top_rollouts_ > 1);

  std_dev /= this->top_rollouts_ - 1;
  for (int i = 0; i < std_dev.rows(); ++i) {
    for (int j = 0; j < std_dev.cols(); ++j) {
      std_dev(i, j) = std::sqrt(std_dev(i, j));
    }
  }
  return std_dev;
}

// Returns the dist between the robot and the target point on the path (squared, computed faster)
double MPC::dist_to_setpoint_() {
  return ((robot_pos_[0] - path_[path_ind_][0]) * (robot_pos_[0] - path_[path_ind_][0]) +
          (robot_pos_[1] - path_[path_ind_][1]) * (robot_pos_[1] - path_[path_ind_][1]));
}

// Given a modeled sample, calculate the cost (Input has shape (horizon_length, 5) where the columns are x, y, theta, linear vel, angular vel)
double MPC::calculate_cost_(Eigen::MatrixXd rollout) {
  SWRI_PROFILE("calculate-cost");

  double cost = 0;
  std::vector<double> robot_pos = this->robot_pos_;
  std::vector<double> goal = this->goal_;

  // Sum up the cost for each horizon step
  for (int i = 0; i < rollout.rows(); ++i) {
    Eigen::RowVectorXd position = rollout.row(i); 

    // linear- linear dist squared between this position and the robot
    cost += this->w_linear_ * ((position(0) - robot_pos[0]) * (position(0) - robot_pos[0]) +
                               (position(1) - robot_pos[1]) * (position(1) - robot_pos[1]));

    // angular- angular difference (squared) between this position and the robot
    cost += this->w_angular_ * (position(2) - robot_pos[2]) * (position(2) - robot_pos[2]);

    // waypoint- the distance between this position and a target point on the path (corrsponding to horizon step)
    int path_cost_i = std::min(path_ind_ + i, (int)path_.size() - 1);
    cost += this->w_waypoint_ *
            ((position(0) - path_[path_cost_i][0]) * (position(0) - path_[path_cost_i][0]) +
             (position(1) - path_[path_cost_i][1]) * (position(1) - path_[path_cost_i][1]));

    // occupied- If this position is an obstacle on the map
    cost += this->w_occupied_ * check_collision_(position);
  }

  return cost;
}

// Given a generated sample (lin/angular velocities for each timestep), model the robot to generate
// the positions for each timestep. Returns a modeled sample (of shape (horizon_length, 5) where the columns are x, y, theta, linear vel, angular vel)
Eigen::MatrixXd MPC::calculate_model_(Eigen::MatrixXd actions) {
  SWRI_PROFILE("calculate-model");

  // Initialize the current position with x, y, theta
  Eigen::VectorXd current_pos(3); 
  current_pos << this->robot_pos_[X_POS], this->robot_pos_[Y_POS], this->robot_pos_[YAW];
  Eigen::MatrixXd model(this->horizon_length_, 5); // x, y, theta, linear vel, angular vel

  // For each row, save the position, and then update it with the given linear/angular velocities
  for (int i = 0; i < this->horizon_length_; ++i) {
    model(i, X_POS) = current_pos(X_POS);
    model(i, Y_POS) = current_pos(Y_POS);
    model(i, YAW) = current_pos(YAW);
    model(i, MODEL_LINEAR_VEL) = actions(i, SAMPLE_LINEAR_VEL);
    model(i, MODEL_ANGULAR_VEL) = actions(i, SAMPLE_ANGULAR_VEL);

    current_pos(X_POS) += std::cos(current_pos(YAW)) * this->delta_time_ * actions(i, SAMPLE_LINEAR_VEL);
    current_pos(Y_POS) += std::sin(current_pos(YAW)) * this->delta_time_ * actions(i, SAMPLE_LINEAR_VEL);
    current_pos(YAW) += this->delta_time_ * actions(i, SAMPLE_ANGULAR_VEL);
  }

  return model;
}

// Used to sort modeled states by their cost
bool comparator(std::pair<Eigen::MatrixXd, double> a, std::pair<Eigen::MatrixXd, double> b) {
  return a.second < b.second;
}

// The main function for MPC- given the current robot position and path, find the best velocities to follow the path
void MPC::calculate_velocity() {
  SWRI_PROFILE("calculate-velocity");

  // If not yet enabled, don't do anything
  if (this->robot_pos_.empty() || !this->enabled_) {
    return;
  }

  // When there's no path, but MPC is still running, stop the robot
  if (this->path_.empty() ) {
    publish_velocity(0, 0);
    return;
  }

  int horizon_length = this->horizon_length_;

  // Initilaize the means and standard deviations to 0 and 1 respectively
  Eigen::MatrixXd means = Eigen::MatrixXd::Zero(horizon_length, 2);    // v, omega
  Eigen::MatrixXd std_devs = Eigen::MatrixXd::Ones(horizon_length, 2); // v, omega

  int iterations = this->iterations_;
  int rollout_count = this->rollout_count_;
  int top_rollouts = this->top_rollouts_;

  // Repeat the process for a number of iterations
  for (int i = 0; i < iterations; ++i) {
    // Generate a variety of random samples (linear/angular and velocities)
    std::vector<Eigen::MatrixXd> random_actions = normal_distribute_(means, std_devs, rollout_count_);

    // x, y, theta, v, omega for each horizon step; paired with cost
    std::vector<std::pair<Eigen::MatrixXd, double>> rollouts(rollout_count_); 

    // Fill each rollout by modeling the generated velocities and calculating the cost
    for (int j = 0; j < rollout_count_; ++j) {
      Eigen::MatrixXd state = calculate_model_(random_actions[j]);
      rollouts[j] = std::make_pair(state, calculate_cost_(state));
    }

    // Sort the modeled states by their cost
    std::sort(rollouts.begin(), rollouts.end(), comparator);

    // If this is the last iteration, publish the velocities of the best state (from the first timestep)
    if (i == iterations - 1) {
      publish_velocity(rollouts[0].first.coeff(0, MODEL_LINEAR_VEL), rollouts[0].first.coeff(0, MODEL_ANGULAR_VEL));
    }
    // Otherwise, update the means/stddevs with the top rollouts and repeat 
    else {
      means = mean_(rollouts);
      std_devs = std_dev_(rollouts);
    }
  }
  // Update the target point on the path if needed
  this->update_setpoint_();
}
