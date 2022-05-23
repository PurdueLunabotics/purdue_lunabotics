#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <lunabot_msgs/Drivetrain.h>

static float limit(float val, float min, float max) {
    if(val < min) {
        return min;
    }
    else if(val > max) {
        return max;
    }
    else {
        return val;
    }
}

class DiffDriveNode {

    private:
        ros::NodeHandle nh_;
        ros::Subscriber cmd_vel_sub_;
        ros::Publisher drivetrain_pub_;

        float lin_vel_;
        float ang_vel_;

        float wheel_separation_;
        float wheel_diameter_;
        float max_wheel_speed_;

        void cmd_vel_cb(const geometry_msgs::Twist& msg) {
            float right = (lin_vel_ + ang_vel_ * wheel_separation_/2.0) / (wheel_diameter_ / 2.0);
            float left = (lin_vel_ - ang_vel_ * wheel_separation_/2.0) / (wheel_diameter_ / 2.0);
            lunabot_msgs::Drivetrain drive_msg;
            drive_msg.left = limit(left / max_wheel_speed_,-1,1);
            drive_msg.right = limit(right / max_wheel_speed_,-1,1);
            drivetrain_pub_.publish(drive_msg);
        }

    public:
        DiffDriveNode() : lin_vel_(0), ang_vel_(0) {
            ros::param::get("wheel_separation", wheel_separation_);
            ros::param::get("wheel_radius", wheel_diameter_);
            ros::param::get("max_wheel_speed", max_wheel_speed_);
            std::string cmd_vel_tpc, drivetrain_tpc;
            ros::param::get("cmd_vel_topic", cmd_vel_tpc);
            ros::param::get("drivetrain_topic", drivetrain_tpc);
            cmd_vel_sub_ = nh_.subscribe(cmd_vel_tpc, 10, &DiffDriveNode::cmd_vel_cb,this);
            drivetrain_pub_ = nh_.advertise<lunabot_msgs::Drivetrain>(drivetrain_tpc,2);
        }
};