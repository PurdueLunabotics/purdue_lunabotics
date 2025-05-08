#include <costmap_2d/array_parser.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "costmap_node");
  tf2_ros::Buffer tf_buff(ros::Duration(5));
  tf2_ros::TransformListener tf(tf_buff);

  costmap_2d::Costmap2DROS costmap("global_costmap", tf_buff);
  bool isRunning = true;
  while (true) {
    ros::spinOnce();
    bool prevRunning = isRunning;
    ros::param::get("/costmap_enabled", isRunning);
    if (!isRunning && prevRunning) {
      ROS_INFO("costmap paused\n");
      costmap.pause();
    } else if(isRunning && !prevRunning) {
      ROS_INFO("costmap resumed\n");
      costmap.resume();
    }
  }
  return 0;
}
