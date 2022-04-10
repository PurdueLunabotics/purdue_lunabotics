#include <lunabot_nav/mpc.h>

MPC::MPC(ros::NodeHandle* nh, int rollout_count, int top_rollouts, int iterations, double w_linear, double w_angular, double w_goal, double w_line, double w_occupied, int horizon_length, double delta_time) {
    this->velocity_pub = nh->advertise<geometry_msgs::TwistStamped>("lunabot_nav/cmd_vel", 10);

    //TODO find names of topics
    this->grid_sub = nh->subscribe("/projected_map", 10, &MPC::update_grid, this);
    this->path_sub = nh->subscribe("/lunabot_nav/path_generator", 10, &MPC::update_path, this); //Convert to Map
    this->goal_sub = nh->subscribe("/goal", 10, &MPC::update_goal, this);
    this->robot_pos_sub = nh->subscribe("/odom", 10, &MPC::update_robot_pos, this); //Convert to Map

    this->rollout_count = rollout_count;
    this->top_rollouts = top_rollouts;
    this->iterations = iterations;
    this->w_linear = w_linear;
    this->w_angular = w_angular;
    this->w_goal = w_goal;
    this->w_line = w_line;
    this->w_occupied = w_occupied;
    this->horizon_length = horizon_length;
    this->delta_time = delta_time;
}

bool MPC::check_collision(Eigen::RowVectorXd pos) {
    //TODO finish or use Raghava's Code
    return false;
}

double MPC::find_closest_distance(Eigen::RowVectorXd pos) { 
    std::vector<std::vector<double>> path = this->path;

    if(path.empty()) {
        return -1;
    }

    double min_dist = -1;

    std::vector<double> point = path[0];
    for(int i = 1; i < path.size(); ++i) {
        std::vector<double> new_point = path[i];
        
        //Building Parametric Lines
        double x_vel = new_point[0] - point[0];
        double y_vel = new_point[1] - point[1];
        
        double t = (x_vel * (pos(0) - point[0]) + y_vel * (pos(1) - point[1])) / (x_vel * x_vel + y_vel * y_vel); //Finding time where line is closest to point

        //Scaling to be on the line segment
        if (t < 0) {
            t = 0;
        } else if(t > 1) {
            t = 1;
        }

        double x_closest = point[0] + t * x_vel;
        double y_closest = point[1] + t * y_vel;

        double distance = std::sqrt((x_closest - pos(0)) * (x_closest - pos(0)) + (y_closest - pos[1]) * (y_closest  - pos(1)));

        if(min_dist == -1 || distance < min_dist) {
            min_dist = distance;
        }

        point = new_point;
    }
    return min_dist;
}

void MPC::update_grid(const nav_msgs::OccupancyGrid& grid) {
    //TODO format based on what collision needs
}

void MPC::update_path(const nav_msgs::Path& path) {
    ROS_INFO("PATH");
    this->path.clear();
    for(int i = 0; i < path.poses.size(); ++i) {
        geometry_msgs::Point position = path.poses[i].pose.position;
        std::vector<double> pos;
        pos.push_back(position.x);
        pos.push_back(position.y);
        this->path.push_back(pos);
    }
}

void MPC::update_goal(const geometry_msgs::PoseStamped& goal) {
    ROS_INFO("GOAL");
    geometry_msgs::Point position = goal.pose.position;
    this->goal.clear();
    this->goal.push_back(position.x);
    this->goal.push_back(position.y);
}

void MPC::update_robot_pos(const nav_msgs::Odometry& odometry) {
    // ROS_INFO("ROBOT_POS");
    geometry_msgs::Point position = odometry.pose.pose.position;
    this->robot_pos.clear();
    this->robot_pos.push_back(position.x);
    this->robot_pos.push_back(position.y);
    this->robot_pos.push_back(position.z); //TODO make angle
}

void MPC::publish_velocity(double linear, double angular) {
    ROS_INFO("PUBLISHING");
    geometry_msgs::TwistStamped twist;
    twist.header.frame_id = "base_link";
    twist.header.stamp = ros::Time::now();
    twist.twist.linear.x = linear;
    twist.twist.angular.z = angular;
    //TODO Stamping stuff idk how it works
    this->velocity_pub.publish(twist);
}

std::vector<Eigen::MatrixXd> MPC::normal_distribute(Eigen::MatrixXd means, Eigen::MatrixXd std_devs, int count) {
    std::vector<Eigen::MatrixXd> random_vels(count);
    for(int i = 0; i < count; ++i) {
        Eigen::MatrixXd random_vel(means.rows(), means.cols());

        for(int j = 0; j < means.rows(); ++j) {
            for(int k = 0; k < means.cols(); ++k) {
                std::default_random_engine generator;
                std::normal_distribution<double> distribution(means(j,k), std_devs(j, k));
                random_vel(j, k) = distribution(generator);
            }
        }
        random_vels[i] = random_vel;
    }
    return random_vels;
}

Eigen::MatrixXd MPC::mean(std::vector<std::pair<Eigen::MatrixXd, double>> rollouts) {
    Eigen::MatrixXd mean = Eigen::MatrixXd::Zero(this->horizon_length, 2);
    for(int i = 0; i < rollouts.size(); ++i) {
        for(int j = 0; j < rollouts[i].first.rows(); ++j) {
            mean(j, 0) += rollouts[i].first(j, 0);
            mean(j, 1) += rollouts[i].first(j, 1);
        }
    }
    return mean / rollouts.size();
}

Eigen::MatrixXd MPC::std_dev(std::vector<std::pair<Eigen::MatrixXd, double>> rollouts) {
    Eigen::MatrixXd temp_mean = mean(rollouts);
    Eigen::MatrixXd std_dev = Eigen::MatrixXd::Zero(this->horizon_length, 2);
    for(int i = 0; i < rollouts.size(); ++i) {
        for(int j = 0; j < rollouts[i].first.rows(); ++j) {
            std_dev(j, 0) += (rollouts[i].first(j, 0) - temp_mean(j, 0)) * (rollouts[i].first(j, 0) - temp_mean(j, 0));
            std_dev(j, 1) += (rollouts[i].first(j, 1) - temp_mean(j, 1)) * (rollouts[i].first(j, 1) - temp_mean(j, 1));
        }
    }
    std_dev /= rollouts.size() - 1;
    for(int i = 0; i < std_dev.rows(); ++i) {
        for(int j = 0; j < std_dev.cols(); ++j) {
            std_dev(i, j) = std::sqrt(std_dev(i, j));
        }
    }
    return std_dev;
}

double MPC::calculate_cost(Eigen::MatrixXd rollout) {
    double cost = 0;
    std::vector<double> robot_pos = this-> robot_pos;
    std::vector<double> goal = this->goal;
    for(int i = 0; i < rollout.rows(); ++i) {
        Eigen::RowVectorXd position = rollout.row(i);
        cost += this->w_linear * ((position(0) - robot_pos[0]) * (position(0) - robot_pos[0]) + (position(1) - robot_pos[1]) * (position(1) - robot_pos[1]));
        cost += this->w_angular * (position(0) - robot_pos[0]) * (position(0) - robot_pos[0]);
        cost += this->w_goal * ((position(0) - goal[0]) * (position(0) - goal[0]) + (position(1) - goal[1]) * (position(1) - goal[1]));
        cost += this->w_line * find_closest_distance(position);
        //cost += this->w_occupied * check_collision(position);
    }
    return cost;
}

Eigen::MatrixXd MPC::calculate_model(Eigen::MatrixXd actions) {
    Eigen::VectorXd current_pos(3); //3 x 1 mat
    current_pos << this->robot_pos[0], this->robot_pos[1], this->robot_pos[2];
    Eigen::MatrixXd A(3, 3);
    A << 
    1, 0, 0,
    0, 1, 0,
    0, 0, 1;
    Eigen::MatrixXd model(this->horizon_length, 3); //x, y, theta
    for(int i = 0; i < this->horizon_length; ++i) {
        model(i, 0) = current_pos(0);
        model(i, 1) = current_pos(1);
        model(i, 2) = current_pos(2);

        Eigen::MatrixXd B(3, 2);
        B << 
        std::cos(current_pos(2)) * this->delta_time, 0,
        std::sin(current_pos(2)) * this->delta_time, 0,
        0, this->delta_time; //TODO test if this model is valid

        current_pos = A * current_pos + B * actions.row(i).transpose();
    }

    return model;

}

bool comparator(std::pair<Eigen::MatrixXd, double> a, std::pair<Eigen::MatrixXd, double> b) {
    return a.second < b.second;
}

void MPC::calculate_velocity() {
    if(this->robot_pos.empty() || this->path.empty() || this->goal.empty()) {
        ROS_INFO("Waiting for Data");
        return;
    }
    ROS_INFO("HERE");
    int horizon_length = this-> horizon_length;
    Eigen::MatrixXd means = Eigen::MatrixXd::Zero(horizon_length, 2); //v, omega
    Eigen::MatrixXd std_devs = Eigen::MatrixXd::Ones(horizon_length, 2); //v, omega
    int iterations = this->iterations;
    int rollout_count = this->rollout_count;
    int top_rollouts = this->top_rollouts;
    std::cout << "2";
    for(int i = 0; i < iterations; ++i) {
        std::vector<Eigen::MatrixXd> random_actions = normal_distribute(means, std_devs, rollout_count);
        std::vector<std::pair<Eigen::MatrixXd, double>> rollouts(rollout_count); //x, y, theta, v, omega for each horizon step paired with cost

        for(int j = 0; j < rollout_count; ++j) {
            Eigen::MatrixXd state = calculate_model(random_actions[j]);
            rollouts[j] = std::make_pair(state, calculate_cost(state));
        }


        //Getting best options
        std::sort(rollouts.begin(), rollouts.end(), comparator);
        std::vector<std::pair<Eigen::MatrixXd, double>> top_rollouts(rollouts.begin(), rollouts.begin() + 1);
        if(i == iterations - 1) {
            publish_velocity(top_rollouts[0].first.coeff(0, 3), top_rollouts[0].first.coeff(0, 4));
        } else {
            means = mean(top_rollouts);
            std_devs = std_dev(top_rollouts);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_node");
    ros::NodeHandle nh;
    double frequency = 20;

    MPC mpc(&nh, 1000, 10, 100, 0.1, 0.1, 0.1, 0.1, 0.9, 5, 1.0/frequency);

    ros::Rate rate(frequency);
    while(ros::ok()) {
        ros::spinOnce();
        mpc.calculate_velocity();
        rate.sleep();
    }
}