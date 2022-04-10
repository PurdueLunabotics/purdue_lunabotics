#include <lunabot_nav/mpc.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_node");
    ros::NodeHandle nh;
    MPC mpc(&nh);
    double frequency;
    ros::param::get("~frequency", frequency);
    ros::Rate rate(frequency);
    while(ros::ok()) {
        ros::spinOnce();
        mpc.calculate_velocity();
        rate.sleep();
    }
}