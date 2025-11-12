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
#include <algorithm>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
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

std::pair<uint32_t, uint32_t> convert_to_grid_coords(nav_msgs::msg::MapMetaData metadata, geometry_msgs::msg::Pose pose) {
  assert(metadata.origin.orientation.x < std::numeric_limits<double>::epsilon());
  assert(metadata.origin.orientation.y < std::numeric_limits<double>::epsilon());
  assert(metadata.origin.orientation.z < std::numeric_limits<double>::epsilon());

  assert(std::abs(metadata.origin.orientation.w - 1) < std::numeric_limits<double>::epsilon());
  return std::pair((pose.position.x - metadata.origin.position.x) / metadata.resolution,
      (pose.position.y - metadata.origin.position.y) / metadata.resolution);
}

geometry_msgs::msg::Pose convert_to_real_coords(nav_msgs::msg::MapMetaData metadata, uint32_t x, uint32_t y) {
  assert(metadata.origin.orientation.x < std::numeric_limits<double>::epsilon());
  assert(metadata.origin.orientation.y < std::numeric_limits<double>::epsilon());
  assert(metadata.origin.orientation.z < std::numeric_limits<double>::epsilon());
  geometry_msgs::msg::Pose pose;

  pose.position.x = x * metadata.resolution + metadata.origin.position.x;
  pose.position.y = y * metadata.resolution + metadata.origin.position.y;
  pose.position.z = 0;

  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  return pose;
}

class DijkstraNode : public rclcpp::Node {
  OdometryMsg odom;
  PoseStampedMsg goal;
  OccupancyGridMsg map;

  rclcpp::Subscription<OdometryMsg>::SharedPtr odom_sub;
  rclcpp::Subscription<PoseStampedMsg>::SharedPtr goal_sub;
  rclcpp::Subscription<OccupancyGridMsg>::SharedPtr map_sub;
  rclcpp::Subscription<OccupancyGridUpdateMsg>::SharedPtr update_sub;
  rclcpp::Publisher<PathMsg>::SharedPtr path_pub;
  rclcpp::TimerBase::SharedPtr timer;

  public:
    DijkstraNode() : rclcpp::Node("dijkstra_node") {
      odom_sub = create_subscription<OdometryMsg>("/rtabmap/odom", 10, [this] (OdometryMsg value) {
          this->odom = value;
      });
      goal_sub = create_subscription<PoseStampedMsg>("/goal", 10, [this] (PoseStampedMsg value) {
          this->goal = value;
      });
      map_sub = create_subscription<OccupancyGridMsg>("/maps/costmap_node/global_costmap/costmap", 10, [this] (OccupancyGridMsg value) {
          this->map = value;
      });
      update_sub = create_subscription<OccupancyGridUpdateMsg>("/maps/costmap_node/global_costmap/costmap_updates", 10, [this] (OccupancyGridUpdateMsg value) {
          for (int x = 0; (uint32_t) x < value.width; x++) {
            for (int y = 0; (uint32_t) y < value.height; y++) {
              if (x + value.x < 0 || (uint32_t) (x + value.x) >= this->map.info.width ||
                  y + value.y < 0 || (uint32_t) (y + value.y) >= this->map.info.height) {
                continue;
              }
              this->map.data[(x + value.x) + (y + value.y) * this->map.info.width] = value.data[x + y * value.width];
            }
          }
      });
      path_pub = create_publisher<PathMsg>("/test_path", 10);
      timer = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&DijkstraNode::plan_path, this));
    }
  
  private:
    void plan_path() {
      std::priority_queue<Vertex, std::vector<Vertex>, std::greater<Vertex>> queue;

      auto hash = [](const Vertex vertex) {
        return std::hash<int>{}(vertex.x) ^ std::hash<int>{}(vertex.y) ^ std::hash<int>{}(vertex.rise) ^ std::hash<int>{}(vertex.run);
      };
      std::unordered_set<Vertex, decltype(hash)> visited(1, hash);

      Vertex initial;
      std::pair<uint32_t, uint32_t> initial_coords = convert_to_grid_coords(map.info, odom.pose.pose);
      initial.x = initial_coords.first;
      initial.y = initial_coords.second;
      initial.prev_x = -1;
      initial.prev_y = -1;

      tf2::Quaternion quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
      tf2::Matrix3x3 rotation_matrix(quaternion);
      double yaw, pitch, roll;
      rotation_matrix.getEulerYPR(yaw, pitch, roll);
      initial.rise = std::cos(pitch) * 100;
      initial.run = std::sin(pitch) * 100;

      queue.push(initial);
      std::pair<int, int> target = convert_to_grid_coords(map.info, goal.pose);

      std::cout << "from x: " << convert_to_grid_coords(map.info, odom.pose.pose).first << " from y: " << convert_to_grid_coords(map.info, odom.pose.pose).second << " to x: " << target.first << " to y: " << target.second << std::endl;

      while (!queue.empty()) {
        if (visited.find(queue.top()) != visited.end()) {
          queue.pop();
          continue;
        }

        if (queue.top().x == target.first && queue.top().y == target.second) {
          // visited.insert(queue.top());
          Vertex coord(queue.top());

          PathMsg path;
          path.header.stamp = rclcpp::Time();
          path.header.frame_id = "odom";

          while (1) {
            PoseStampedMsg pose;

            pose.header.stamp = rclcpp::Time();
            pose.header.frame_id = "odom";
            pose.pose = convert_to_real_coords(map.info, (uint32_t) coord.x, (uint32_t) coord.y);
            path.poses.push_back(pose);
            auto vertex = visited.find(coord.partial_prev());
            if (vertex == visited.end()) {
              std::cout << "failed to find root! x: " << coord.x << " y: " << coord.y << std::endl;
              break;
            } else {
              // std::cout << "found root!" << std::endl;
            }
            coord = *vertex;
          }
          std::reverse(path.poses.begin(), path.poses.end());
          path_pub->publish(path);
          break;
        }

        for (int x_offset = -2; x_offset <= 2; x_offset++) {
          for (int y_offset = -2; y_offset <= 2; y_offset += 4) {
            Vertex neighbor(queue.top());
            neighbor.apply_offset(x_offset, y_offset);

            if (neighbor.x < 0 ||
                neighbor.y < 0 ||
                (uint32_t) neighbor.x >= map.info.width ||
                (uint32_t) neighbor.y >= map.info.height ||
                map.data[(uint32_t) neighbor.x + (uint32_t) neighbor.y * map.info.width] >= 30 ||
                visited.find(neighbor) != visited.end()) {
              continue;
            }
            double dist = std::sqrt(std::pow(x_offset, 2) + std::pow(y_offset, 2));
            if (neighbor.prev_rise != neighbor.rise || neighbor.prev_run != neighbor.run) {
              neighbor.cost += dist * 10.0;
            } else {
              neighbor.cost += dist;
            }

            double dist_to_target = std::abs(neighbor.x - target.first) + std::abs(neighbor.y - target.second);
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
                (uint32_t) neighbor.x >= map.info.width ||
                (uint32_t) neighbor.y >= map.info.height ||
                map.data[(uint32_t) neighbor.x + (uint32_t) neighbor.y * map.info.width] >= 30 ||
                visited.find(neighbor) != visited.end()) {
              continue;
            }
            double dist = std::sqrt(std::pow(x_offset, 2) + std::pow(y_offset, 2));
            if (neighbor.prev_rise != neighbor.rise || neighbor.prev_run != neighbor.run) {
              neighbor.cost += dist * 10.0;
            } else {
              neighbor.cost += dist;
            }

            double dist_to_target = std::abs(neighbor.x - target.first) + std::abs(neighbor.y - target.second);
            neighbor.cost += dist_to_target;

            queue.push(neighbor);
          }
        }

        visited.insert(queue.top());
        queue.pop();
      }
      std::cout << "reached end" << std::endl;
    }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DijkstraNode>());
  rclcpp::shutdown();
  return 0;
}
