#include <ros/ros.h>
#include <lunabot_localization/bt_localization.h>
#include <memory>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

/*
ROS parameters:
bt_UUID1 - UUID of tag 1
bt_UUID2 - UUID of tag 2
bt_UUID3 - UUID of tag 3
*/

class BTLocalizationNode 
{
private:
    //Get settings from params
    std::unique_ptr<BTLocalization> bt;

    ros::Subscriber calibrate_command_subscriber_;
    ros::Subscriber calibrate_command_subscriber_;
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
      //Get UUIDs from parameters, pass them down
      int UUID1, UUID2, UUID3;
      nh->getParam("~beac_UUID1", UUID1);
      nh->getParam("~beac_UUID2", UUID2);
      nh->getParam("~beac_UUID3", UUID3);

      //Get bluetooth tag locations in fixed reference frame (apriltag frame)
      std::vector<double> U, V, W; 
      nh->getParam("~beac1pos", U); // [x,y,z]
      nh->getParam("~beac2pos", V);
      nh->getParam("~beac3pos", W);

      bt.reset(new BTLocalization((uint16_t) UUID1, (uint16_t) UUID2, (uint16_t) UUID3, U, V, W));

      current_pos_publisher_ = nh->advertise<geometry_msgs::PoseStamped>("current_pos", 10);
      current_pos_timer_ = nh->createTimer(ros::Duration(1.0 / publish_current_pos_frequency_), &BTLocalizationNode::publishCurrentPos, this);
      calibrate_command_subscriber_ = nh->subscribe("bt_calibrate_command", 3, &BTLocalizationNode::callbackCalibrate, this);
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

    void apriltagCb(apriltag_ros::AprilTagDetectionArrayConstPtr msg) //For calibrating strength of bluetooth tags
    {
        //Get the node positions from parameters
        //Calculate ranges
        float r1 = sqrt(btt1.x * btt1.x + btt1.y * btt1.y + btt1.z * btt1.z);
        float r2 = sqrt(btt2.x * btt2.x + btt2.y * btt2.y + btt2.z * btt2.z);
        float r3 = sqrt(btt3.x * btt3.x + btt3.y * btt3.y + btt3.z * btt3.z);
        //Calibrate each tag
        bt->calibrate(r1, r2, r3);
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
