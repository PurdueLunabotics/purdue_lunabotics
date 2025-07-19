#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("costmap_node");
    // tf2_ros::Buffer tf_buff(rclcpp::Duration(5,0));
    // tf2_ros::TransformListener tf(tf_buff);

  bool enabled = true;
  bool isRunning = true;

  std::string node_name = "global_costmap";
  std::string parent_ns = "maps";

  nav2_costmap_2d::Costmap2DROS costmap(node_name);

  while (rclcpp::ok()) {
    rclcpp::spin(node);

    // check to see if we should be enabled
    

    // if we are currently running, but shouldn't be enabled, try to disable the map (pause it)
    // if (isRunning && !enabled) {

    //   /*
    //    * NOTE:  Do Not use this yet; pausing costmap seems to result in a blank map even with this check
    //    */

    //   // check to make sure that it's not all zeroes.
    //   nav2_costmap_2d::Costmap2D* map = costmap.getCostmap();
    //   int xMax = map->getSizeInCellsX();
    //   int yMax = map->getSizeInCellsY();
    //   std::cout << "****xmax " << xMax << "   ***** ymax" << yMax << "\n";

    //   bool allZeroes = true;

    //   for (int x = 0; x < xMax; x++) {
    //     for (int y = 0; y < yMax; y++) {
    //       unsigned int cost = map->getCost(x, y);
    //       std::cout << cost << " ";
    //       if (cost != 0) {
    //         allZeroes = false;
    //         break;
    //       }
    //     }
    //     if (!allZeroes) {
    //       break;
    //     }
    //   }

    //   if (!allZeroes) {
    //     RCLCPP_INFO(node->get_logger(), "costmap paused\n");
    //     costmap.pause();
    //     isRunning = false;
    //   }
    //   else {
    //     RCLCPP_INFO(node->get_logger(), "Costmap tried to pause, was all 0\n");
    //   }

    // } else if (enabled && !isRunning) {
    //   RCLCPP_INFO(node->get_logger(), "costmap resumed\n");
    //   costmap.resume();
    //   isRunning = true;
    // }
  }
  return 0;
}
