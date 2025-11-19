#include "dstar.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <thread>

using std::placeholders::_1;


class DstarNode
{
public:
  DstarNode()
  {
    node = rclcpp::Node::make_shared("dstar_node");
  }

  void dstar_loop()
  {
    // rclcpp::param::get("/odom_topic", odom_topic);
    // rclcpp::param::get("/nav_goal_topic", goal_topic);
    // rclcpp::param::get("/nav/map_topic", map_topic);
    // rclcpp::param::get("/nav/map_update_topic", map_update_topic);
    // rclcpp::param::get("/nav/dstar_node/path_sampling_rate", path_sampling_rate); // Take every<n-th> point from the path
    // rclcpp::param::get("/nav/global_path_topic", path_topic);
    // rclcpp::param::get("/nav/occ_threshold", occupancy_threshold);

    // rclcpp::QoS *qos = new rclcpp::QoS(rclcpp::KeepAll());
    int qos = 10;
    // odom_topic = "/rtabmap/odom";
    odom_topic = "/rtabmap/position";
    goal_topic = "/goal";
    map_topic = "/maps/costmap_node/global_costmap/costmap";
    map_update_topic = "/maps/costmap_node/global_costmap/costmap_updates";
    path_sampling_rate = 5;
    path_topic = "/nav/global_path";
    occupancy_threshold = 50;

    int frequency = 3; // hz

    auto map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic, qos, std::bind(&DstarNode::grid_callback, this, _1));
    auto map_update_sub = node->create_subscription<map_msgs::msg::OccupancyGridUpdate>(map_update_topic, qos, std::bind(&DstarNode::grid_update_callback, this, _1));
    auto odom_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(odom_topic, qos, std::bind(&DstarNode::position_callback, this, _1));
    auto goal_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(goal_topic, qos, std::bind(&DstarNode::goal_callback, this, _1));
    path_pub = node->create_publisher<nav_msgs::msg::Path>(path_topic, 10);
    auto planning_enabled_subscriber = node->create_subscription<std_msgs::msg::Bool>("/nav/planning_enabled", qos, std::bind(&DstarNode::enable_callback, this, _1));
    auto costmap_enabled_subscriber = node->create_subscription<std_msgs::msg::Bool>("/costmap_enabled", qos, std::bind(&DstarNode::costmap_enabled_callback, this, _1));

    rclcpp::Rate rate(frequency);
    while (rclcpp::ok())
    {
      if (!dstar_init && map_init && goal_init && pose_init)
      {
        RCLCPP_DEBUG(node->get_logger(), "INIT DSTAR");
        dstar = Dstar(goal, pose, map, resolution, x_offset, y_offset, occupancy_threshold);
        dstar_init = true;
        RCLCPP_DEBUG(node->get_logger(), "Found Path");
        publish_path(dstar.find_path());
        goal_update_needed = false;
        grid_update_needed = false; // dif from python, don't update map if not strictly needed
        continue;
      }

      if (dstar_init)
      {
        if (goal_update_needed)
        {
          RCLCPP_DEBUG(node->get_logger(), "Goal Updating");
          // If we've gotten a new goal, reset the dstar data and find a new path
          dstar = Dstar(goal, pose, map, resolution, x_offset, y_offset, occupancy_threshold);
          RCLCPP_DEBUG(node->get_logger(), "Found Path");
          publish_path(dstar.find_path());
          goal_update_needed = false;
          grid_update_needed = false; // dif from python, don't update map if not strictly needed

          continue;
        }

        if (grid_update_needed)
        {
          // If the map has changed, update Dstar with the new position and map, then find and publish a new path
          dstar.update_position(pose);

          RCLCPP_DEBUG(node->get_logger(), "Dstar grid update");

          publish_path(dstar.update_map(map, x_offset, y_offset));
          grid_update_needed = false;

          continue;
        }
      }
      rclcpp::spin_some(node);
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
  int empty_paths = 0; // how many empty paths in a row have been published

  std::mutex map_lock; // NOTE: this mutex was needed in python, might not be needed in C++ because roscpp callbacks are single threaded

  std::shared_ptr<rclcpp::Node> node;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path_<std::allocator<void>>, std::allocator<void>>> path_pub;

  std::string odom_topic;
  std::string goal_topic;
  std::string map_topic;
  std::string map_update_topic;
  std::string path_topic;
  int path_sampling_rate;
  float occupancy_threshold;

  bool planning_enabled = true;
  bool costmap_enabled = true;

  void enable_callback(const std_msgs::msg::Bool &msg)
  {
    this->planning_enabled = msg.data;
  }

  void costmap_enabled_callback(const std_msgs::msg::Bool &msg)
  {
    this->costmap_enabled = msg.data;
  }

  // Updates the map given a new occupancy grid.Update the flag such that dstar will update the map.
  void grid_callback(const nav_msgs::msg::OccupancyGrid &data)
  {
    // RCLCPP_INFO(node->get_logger(), "Got Grid");
    map_lock.lock();

    bool map_ok = false;
    for (int i = 0; i < data.info.width * data.info.height; i++)
    {
      if (data.data[i] != 0)
      {
        map_ok = true;
        break;
      }
    }

    uint32_t width = data.info.width;
    uint32_t height = data.info.height;

    map = std::vector<std::vector<int>>(height, std::vector<int>(width));

    for (int i = 0; i < height; ++i)
    {
      for (int j = 0; j < width; ++j)
      {
        map[i][j] = data.data[i * width + j];
      }
    }

    map_init = true;

    resolution = data.info.resolution;
    x_offset = data.info.origin.position.x;
    y_offset = data.info.origin.position.y;

    if (!map_ok)
    { // if the map is all zeroes, dont't update dstar with it
      map_lock.unlock();
      return;
    }

    grid_update_needed = true;

    map_lock.unlock();
  }

  // Update the grid given the occupancy grid update (applied on top of the current grid). Also update the flag for dstar to update the map.
  void grid_update_callback(const map_msgs::msg::OccupancyGridUpdate &data)
  {
    // RCLCPP_INFO(node->get_logger(), "Got Grid Update");
    map_lock.lock();

    bool map_ok = false;
    for (int i = 0; i < data.width * data.height; i++)
    {
      if (data.data[i] != 0)
      {
        map_ok = true;
        break;
      }
    }
    if (!map_ok)
    {
      map_lock.unlock();
      return;
    }

    int index = 0;
    for (int i = data.y; i < data.y + data.height; i++)
    {
      for (int j = data.x; j < data.x + data.width; j++)
      {
        if (0 <= i && i < map.size() && 0 <= j && j < map[0].size())
        { // prevents issue where dstar still processing and new map is loaded in
          map[i][j] = data.data[index];
          index++;
        }
      }
    }

    map_init = true;
    grid_update_needed = true;

    map_lock.unlock();
  }

  void position_callback(const geometry_msgs::msg::PoseStamped &data)
  {
    // RCLCPP_INFO(node->get_logger(), "Got Pose");
    pose.x = data.pose.position.x;
    pose.y = data.pose.position.y;
    pose_init = true;
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped &data)
  {
    RCLCPP_DEBUG(node->get_logger(), "Got Goal");
    goal.x = data.pose.position.x;
    goal.y = data.pose.position.y;
    goal_update_needed = true;
    goal_init = true;
  }

  // Publish the calculated path using the class's path publisher.
  // Reduces the size / complexity of the path using the path sampling rate
  void publish_path(const std::vector<real_world_point> &path_data)
  {
    RCLCPP_DEBUG(node->get_logger(), "Publishing path");
    nav_msgs::msg::Path path;
    // rclcpp::Clock::SharedPtr clock = node.get_clock()
    path.header.stamp = rclcpp::Time();
    path.header.frame_id = "odom"; // Set the frame of reference

    if (path_data.size() == 0)
    {
      // Only publish some empty paths, avoid overwhelming traversal.
      if (empty_paths % 20 != 0)
      {
        return;
      }

      empty_paths++;
    }
    else
    {
      // when it's not empty, restart this counter.
      empty_paths = 0;
    }

    for (size_t index = 0; index < path_data.size(); ++index)
    {
      if (index % path_sampling_rate == 0 || index == path_data.size() - 1)
      {
        // Sample every <path_sampling_rate_> points (+ the last one)
        geometry_msgs::msg::PoseStamped path_pose;
        path_pose.header.stamp = rclcpp::Time();
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

    bool mapAllZero = true;
    for (int i = 0; i < this->map.size(); i++)
    {
      for (int j = 0; j < this->map[i].size(); j++)
      {
        if (this->map[i][j] != 0)
        {
          mapAllZero = false;
          break;
        }
      }
    }

    // if the map is all zeroes, and the costmap is still on, don't publish paths
    if (mapAllZero && this->costmap_enabled)
    {
      return;
    }

    if (this->planning_enabled)
    {
      path_pub->publish(path);
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  DstarNode dstar_node;
  dstar_node.dstar_loop();
  return 0;
}
