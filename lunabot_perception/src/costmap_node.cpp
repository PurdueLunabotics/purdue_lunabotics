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

  bool enabled = true;
  bool isRunning = true;

  costmap_2d::Costmap2DROS costmap("global_costmap", tf_buff);

  while (ros::ok()) {
    ros::spinOnce();

    // check to see if we should be enabled
    ros::param::get("/costmap_enabled", enabled);

    // if we are currently running, but shouldn't be enabled, try to disable the map (pause it)
    if (isRunning && !enabled) {

      /*
       * NOTE:  Do Not use this yet; pausing costmap seems to result in a blank map even with this check
       */

      // check to make sure that it's not all zeroes.
      costmap_2d::Costmap2D* map = costmap.getCostmap();
      int xMax = map->getSizeInCellsX();
      int yMax = map->getSizeInCellsY();
      std::cout << "****xmax " << xMax << "   ***** ymax" << yMax << "\n";

      bool allZeroes = true;

      for (int x = 0; x < xMax; x++) {
        for (int y = 0; y < yMax; y++) {
          unsigned int cost = map->getCost(x, y);
          std::cout << cost << " ";
          if (cost != 0) {
            allZeroes = false;
            break;
          }
        }
        if (!allZeroes) {
          break;
        }
      }

      if (!allZeroes) {
        ROS_INFO("costmap paused\n");
        costmap.pause();
        isRunning = false;
      }
      else {
        ROS_INFO("Costmap tried to pause, was all 0\n");
      }

    } else if (enabled && !isRunning) {
      ROS_INFO("costmap resumed\n");
      costmap.resume();
      isRunning = true;
    }
  }
  return 0;
}
