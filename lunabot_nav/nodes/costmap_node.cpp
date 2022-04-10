#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/array_parser.h>
#include <geometry_msgs/Point.h>


int main(int argc, char** argv) {
    ros::init(argc,argv,"costmap_node");
    tf2_ros::Buffer tf_buff(ros::Duration(5));
    tf2_ros::TransformListener tf(tf_buff);

    costmap_2d::Costmap2DROS costmap("global_costmap",tf_buff);
    ros::spin();
    return 0;
}
