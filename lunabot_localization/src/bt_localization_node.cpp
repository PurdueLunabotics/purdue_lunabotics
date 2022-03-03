#include <ros/ros.h>
#include <lunabot_localization/bt_localization.h>
#include <memory>
#include <geometry_msgs/PoseStamped.h>

class BTLocalizationNode 
{
private:
    //Get settings from params
    std::unique_ptr<BTLocalization> bt;
    ros::Subscriber bt_command_subscriber;
    ros::Publisher current_pos_publisher_;
    ros::Timer current_pos_timer_;

    double publish_current_pos_frequency_;

public:

    BTLocalizationNode(ros::NodeHandle *nh) 
    {
      /*
      if (!ros::param::get("~publish_current_pos_frequency", publish_current_pos_frequency_)) 
      {
      }
      */

      publish_current_pos_frequency_ = 2.0;
      bt.reset(new BTLocalization);

      current_pos_publisher_ = nh->advertise<geometry_msgs::PoseStamped>("current_pos", 10);
      current_pos_timer_ = nh->createTimer(ros::Duration(1.0 / publish_current_pos_frequency_), &BTLocalizationNode::publishCurrentPos, this);
    }

    void publishCurrentPos(const ros::TimerEvent &event) 
    {
        bt->getPos(); //Update stored position
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = bt->x;  
        pose.pose.position.y = bt->y;  
        pose.pose.position.z = bt->z;  
        pose.pose.orientation.x = 0;  
        pose.pose.orientation.y = 0;  
        pose.pose.orientation.z = 0;  
        pose.pose.orientation.w = 1;  
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "ble_frame";

        current_pos_publisher_.publish(pose);
    }

    void stop()
    {
        //Add cleanup here
    }
};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "bt_localization_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    BTLocalizationNode bt_node = BTLocalizationNode(&nh);
    ROS_INFO("Bluetooth driver is now started");

    ros::waitForShutdown();
    bt_node.stop();
}
