#include "dstar.hpp"
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <thread>

class DstarNode {
public:
  DstarNode() {
  }

  void dstar_loop() {
    ros::param::get("/odom_topic", odom_topic);
    ros::param::get("/nav_goal_topic", goal_topic);
    ros::param::get("/nav/map_topic", map_topic);
    ros::param::get("/nav/map_update_topic", map_update_topic);
    ros::param::get("/nav/dstar_node/path_sampling_rate", path_sampling_rate); // Take every<n-th> point from the path
    ros::param::get("/nav/global_path_topic", path_topic);
    ros::param::get("/nav/occ_threshold", occupancy_threshold);

    int frequency = 3; // hz

    map_sub = nh.subscribe(map_topic, 1, &DstarNode::grid_callback, this);
    map_update_sub = nh.subscribe(map_update_topic, 1, &DstarNode::grid_update_callback, this);
    odom_sub = nh.subscribe(odom_topic, 1, &DstarNode::position_callback, this);
    goal_sub = nh.subscribe(goal_topic, 1, &DstarNode::goal_callback, this);
    path_pub = nh.advertise<nav_msgs::Path>(path_topic, 10, true);
    planning_enabled_subscriber = nh.subscribe("/nav/planning_enabled", 1, &DstarNode::enable_callback, this);

    ros::Rate rate(frequency);
    while (ros::ok()) {
      if (!dstar_init && map_init && goal_init && pose_init) {
        dstar = Dstar(goal, pose, map, resolution, x_offset, y_offset, occupancy_threshold);
        dstar_init = true;
        publish_path(dstar.find_path());
        goal_update_needed = false;
        grid_update_needed = false; // dif from python, don't update map if not strictly needed
        continue;
      }

      if (dstar_init) {
        if (goal_update_needed) {
          // If we've gotten a new goal, reset the dstar data and find a new path
          dstar = Dstar(goal, pose, map, resolution, x_offset, y_offset, occupancy_threshold);
          publish_path(dstar.find_path());
          goal_update_needed = false;
          grid_update_needed = false; // dif from python, don't update map if not strictly needed

          continue;
        }

        if (grid_update_needed) {
          // If the map has changed, update Dstar with the new position and map, then find and publish a new path
          dstar.update_position(pose);

          ROS_DEBUG("Dstar grid update");

          publish_path(dstar.update_map(map, x_offset, y_offset));
          grid_update_needed = false;

          continue;
        }
      }

      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  Dstar dstar;
  std::vector<std::vector<int>> map;
  real_world_point goal;
  real_world_point pose;
  float resolution;   // meters per grid cell
  float x_offset = 0; // real world pose of the point 0, 0 in the grid
  float y_offset = 0;

  bool goal_init = false;
  bool pose_init = false;
  bool map_init = false;
  bool dstar_init = false;
  bool grid_update_needed = false;
  bool goal_update_needed = false;
  bool just_published_empty_path = false;

  std::mutex map_lock;  // NOTE: this mutex was needed in python, might not be needed in C++ because roscpp callbacks are single threaded

  ros::NodeHandle nh;
  ros::Subscriber map_sub;
  ros::Subscriber map_update_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber goal_sub;
  ros::Subscriber planning_enabled_subscriber;
  ros::Publisher path_pub;

  std::string odom_topic;
  std::string goal_topic;
  std::string map_topic;
  std::string map_update_topic;
  std::string path_topic;
  int path_sampling_rate;
  float occupancy_threshold;

  bool planning_enabled = true;

  void enable_callback(const std_msgs::Bool::ConstPtr &msg) {
    this->planning_enabled = msg->data;
  }

  // Updates the map given a new occupancy grid.Update the flag such that dstar will update the map.
  void grid_callback(const nav_msgs::OccupancyGrid::ConstPtr &data) {

    map_lock.lock();

    bool map_ok = false;
    for (int i = 0; i < data->info.width * data->info.height; i++) {
      if (data->data[i] != 0) {
        map_ok = true;
        break;
      }
    }

    uint32_t width = data->info.width;
    uint32_t height = data->info.height;

    map = std::vector<std::vector<int>>(height, std::vector<int>(width));

    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        map[i][j] = data->data[i * width + j];
      }
    }

    map_init = true;

    resolution = data->info.resolution;
    x_offset = data->info.origin.position.x;
    y_offset = data->info.origin.position.y;

    if (!map_ok) { // if the map is all zeroes, dont't update dstar with it
      map_lock.unlock();
      return;
    }

    grid_update_needed = true;

    map_lock.unlock();
  }

  // Update the grid given the occupancy grid update (applied on top of the current grid). Also update the flag for dstar to update the map.
  void grid_update_callback(const map_msgs::OccupancyGridUpdate::ConstPtr &data) {

    map_lock.lock();

    bool map_ok = false;
    for (int i = 0; i < data->width * data->height; i++) {
      if (data->data[i] != 0) {
        map_ok = true;
        break;
      }
    }
    if (!map_ok) {
      map_lock.unlock();
      return;
    }

    int index = 0;
    for (int i = data->y; i < data->y + data->height; i++) {
      for (int j = data->x; j < data->x + data->width; j++) {
        if (0 <= i && i < map.size() && 0 <= j && j < map[0].size()) { // prevents issue where dstar still processing and new map is loaded in
          map[i][j] = data->data[index];
          index++;
        }
      }
    }

    grid_update_needed = true;

    map_lock.unlock();
  }

  void position_callback(const nav_msgs::Odometry::ConstPtr &data) {
    pose.x = data->pose.pose.position.x;
    pose.y = data->pose.pose.position.y;
    pose_init = true;
  }

  void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &data) {
    goal.x = data->pose.position.x;
    goal.y = data->pose.position.y;
    goal_update_needed = true;
    goal_init = true;
  }

  // Publish the calculated path using the class's path publisher.
  // Reduces the size / complexity of the path using the path sampling rate
  void publish_path(const std::vector<real_world_point> &path_data) {
    ROS_DEBUG("Publishing path");

    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "odom"; // Set the frame of reference

    if (path_data.size() == 0) {
      if (!just_published_empty_path) {
        just_published_empty_path = true;
        // and continue on, publish it
      }
      else {
        return; // Don't publish empty path if already published
      }
    }
    else {
      just_published_empty_path = false;
    }

    for (size_t index = 0; index < path_data.size(); ++index) {
      if (index % path_sampling_rate == 0 || index == path_data.size() - 1) {
        // Sample every <path_sampling_rate_> points (+ the last one)
        geometry_msgs::PoseStamped path_pose;
        path_pose.header.stamp = ros::Time::now();
        path_pose.header.frame_id = "odom";

        path_pose.pose.position.x = path_data[index].x;
        path_pose.pose.position.y = path_data[index].y;
        path_pose.pose.position.z = 0.0; // Assuming 2D path, Z is 0

        // Assuming no rotation (quaternion identity)
        path_pose.pose.orientation.x = 0;
        path_pose.pose.orientation.y = 0;
        path_pose.pose.orientation.z = 0;
        path_pose.pose.orientation.w = 1;

        path.poses.push_back(path_pose);
      }
    }

    if (this->planning_enabled) {
      path_pub.publish(path);
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "dstar_node");
  DstarNode dstar_node; 
  dstar_node.dstar_loop();
  return 0;
}