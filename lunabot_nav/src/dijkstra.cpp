#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include <algorithm>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <queue>
#include <utility>

typedef nav_msgs::msg::Odometry OdometryMsg;
typedef geometry_msgs::msg::PoseStamped PoseStampedMsg;
typedef nav_msgs::msg::OccupancyGrid OccupancyGridMsg;
typedef map_msgs::msg::OccupancyGridUpdate OccupancyGridUpdateMsg;
typedef nav_msgs::msg::Path PathMsg;

struct Vertex {
  int x = 0;
  int y = 0;
  int rise = 0;
  int run = 0;
  int prev_x = 0;
  int prev_y = 0;
  int prev_rise = 0;
  int prev_run = 0;
  double cost;

  Vertex() { }

  void apply_offset(int x_offset, int y_offset) {
    prev_x = x;
    prev_y = y;
    prev_rise = rise;
    prev_run = run;
    x += x_offset;
    y += y_offset;
    rise = x_offset;
    run = y_offset;
  }

  Vertex partial_prev() {
    Vertex prev;
    prev.x = prev_x;
    prev.y = prev_y;
    prev.rise = prev_rise;
    prev.run = prev_run;
    return prev;
  }

  bool operator==(const Vertex& other) const {
    return x == other.x && y == other.y && rise == other.rise && run == other.run;
  }

  bool operator>(const Vertex& other) const {
    return cost > other.cost;
  }
};

class DijkstraPlanner : public nav2_core::GlobalPlanner {
  nav2_costmap_2d::Costmap2D *costmap;
  std::string frame_id;
  rclcpp::Logger logger = rclcpp::get_logger("DijkstraPlanner");

  public:
    DijkstraPlanner() { }

    void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override {
      costmap = costmap_ros->getCostmap();
      frame_id = costmap_ros->getGlobalFrameID();
    }

    void cleanup() override {}
    void activate() override {}
    void deactivate() override {}

    nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal) override {
      std::priority_queue<Vertex, std::vector<Vertex>, std::greater<Vertex>> queue;

      auto hash = [](const Vertex vertex) {
        return std::hash<int>{}(vertex.x) ^ std::hash<int>{}(vertex.y) ^ std::hash<int>{}(vertex.rise) ^ std::hash<int>{}(vertex.run);
      };
      std::unordered_set<Vertex, decltype(hash)> visited(1, hash);

      int target_x, target_y;
      costmap->worldToMapNoBounds(goal.pose.position.x, goal.pose.position.y, target_x, target_y);

      Vertex initial;
      int initial_x, initial_y;
      costmap->worldToMapNoBounds(start.pose.position.x, start.pose.position.y, initial_x, initial_y);
      initial.x = initial_x;
      initial.y = initial_y;
      initial.prev_x = -1;
      initial.prev_y = -1;

      tf2::Quaternion quaternion(start.pose.orientation.x, start.pose.orientation.y, start.pose.orientation.z, start.pose.orientation.w);
      tf2::Matrix3x3 rotation_matrix(quaternion);
      double yaw, pitch, roll;
      rotation_matrix.getEulerYPR(yaw, pitch, roll);
      initial.rise = std::cos(pitch) * 100;
      initial.run = std::sin(pitch) * 100;

      queue.push(initial);

      while (!queue.empty()) {
        if (visited.find(queue.top()) != visited.end()) {
          queue.pop();
          continue;
        }

        if (queue.top().x == target_x && queue.top().y == target_y) {
          RCLCPP_INFO(logger, "found target!");
          // visited.insert(queue.top());
          Vertex coord(queue.top());

          PathMsg path;
          path.header.stamp = rclcpp::Time();
          path.header.frame_id = frame_id;

          while (1) {
            PoseStampedMsg pose;

            pose.header.stamp = rclcpp::Time();
            pose.header.frame_id = frame_id;
            costmap->mapToWorld((unsigned int) coord.x, (unsigned int) coord.y, pose.pose.position.x, pose.pose.position.y);
            pose.pose.position.z = 0;

            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.w = 1;
            path.poses.push_back(pose);
            auto vertex = visited.find(coord.partial_prev());
            if (vertex == visited.end()) {
              RCLCPP_WARN(logger, "failed to trace to root!");
              break;
            }
            coord = *vertex;
          }
          std::reverse(path.poses.begin(), path.poses.end());
          return path;
          break;
        }

        for (int x_offset = -2; x_offset <= 2; x_offset++) {
          for (int y_offset = -2; y_offset <= 2; y_offset += 4) {
            Vertex neighbor(queue.top());
            neighbor.apply_offset(x_offset, y_offset);

            if (neighbor.x < 0 ||
                neighbor.y < 0 ||
                (uint32_t) neighbor.x >= costmap->getSizeInCellsX() ||
                (uint32_t) neighbor.y >= costmap->getSizeInCellsY() ||
                costmap->getCost(neighbor.x, neighbor.y) >= 30 ||
                visited.find(neighbor) != visited.end()) {
              continue;
            }
            double dist = std::sqrt(std::pow(x_offset, 2) + std::pow(y_offset, 2));
            if (neighbor.prev_rise != neighbor.rise || neighbor.prev_run != neighbor.run) {
              neighbor.cost += dist * 10.0;
            } else {
              neighbor.cost += dist;
            }

            double dist_to_target = std::abs(neighbor.x - (int)target_x) + std::abs(neighbor.y - (int)target_y);
            neighbor.cost += dist_to_target;

            queue.push(neighbor);
          }
        }

        for (int x_offset = -2; x_offset <= 2; x_offset += 4) {
          for (int y_offset = -1; y_offset <= 1; y_offset++) {
            Vertex neighbor(queue.top());
            neighbor.apply_offset(x_offset, y_offset);

            if (neighbor.x < 0 ||
                neighbor.y < 0 ||
                (uint32_t) neighbor.x >= costmap->getSizeInCellsX() ||
                (uint32_t) neighbor.y >= costmap->getSizeInCellsY() ||
                costmap->getCost(neighbor.x, neighbor.y) >= 30 ||
                visited.find(neighbor) != visited.end()) {
              continue;
            }
            double dist = std::sqrt(std::pow(x_offset, 2) + std::pow(y_offset, 2));
            if (neighbor.prev_rise != neighbor.rise || neighbor.prev_run != neighbor.run) {
              neighbor.cost += dist * 10.0;
            } else {
              neighbor.cost += dist;
            }

            double dist_to_target = std::abs(neighbor.x - (int)target_x) + std::abs(neighbor.y - (int)target_y);
            neighbor.cost += dist_to_target;

            queue.push(neighbor);
          }
        }

        visited.insert(queue.top());
        queue.pop();
      }
      PathMsg path;
      path.poses.clear();
      return path;
    }

};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(DijkstraPlanner, nav2_core::GlobalPlanner)
