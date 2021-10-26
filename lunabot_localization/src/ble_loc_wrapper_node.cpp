#include <ros/ros.h>
#include "manager.h"
#include <memory>
#include <std_msgs/Float32MultiArray.h>

class BTDriverROSWrapper 
{
private:
    //Get settings from params
    std::unique_ptr<Manager> bt;
    ros::Subscriber bt_command_subscriber;
    ros::Publisher current_pos_publisher_;
    ros::Timer current_speed_timer_;
    double publish_current_pos_frequency_;

public:
    if (!ros::param::get("~publish_current_pos_frequency", publish_current_pos_frequency_)) 
    {
        publish_current_pos_frequency_ = 2.0;
    }

    current_pos_publisher = nh->advertise<std_msgs::Float32MultiArray>("current_pos", 10);
    current_pos_timer = nh-> createTimer(ros::Duration(1.0 / publish_current_pos_frequency_), &BTDriverROSWrapper::publishCurrentPos, this);


    BTDriverROSWrapper(ros::NodeHandle *nh) 
    {
        bt.reset(new Manager);
    }

    void publishCurrentPos(const ros::TimerEvent &event) 
    {
        bt->getPos(); //Update stored position
        std_msgs::Float32MultiArray msg;
        msg.data = new float[3];
        msg.data[0] = bt->x;
        msg.data[1] = bt->y;
        msg.data[2] = bt->z;
        current_pos_publisher_.publish(msg);
    }

    void stop()
    {
        //Add cleanup here
    }
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "bt_driver");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    BTDriverROSWrapper bt_wrapper;
    ROS_INFO("Bluetooth driver is now started");

    ros::waitForShutdown();
    bt_wrapper.stop();
}