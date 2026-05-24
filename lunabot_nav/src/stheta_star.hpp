#ifndef STHETA_STAR_HPP
#define STHETA_STAR_HPP

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

typedef geometry_msgs::msg::PoseStamped PoseStampedMsg;
typedef nav_msgs::msg::Path PathMsg;

struct Options {
  double turning_cost;
  double driving_cost;
  double traversal_cost;
  double node_cost;
  double costmap_exponential;
  double max_goal_adjustment_meters; // max distance it will go in a line
  double remapping_cost_change_percent;
};

struct Coord {
  int x;
  int y;

  Coord(int x, int y);
  Coord();
};

struct Vertex {
  Coord coord;
  Coord prev_coord;
  double cost;
  bool visited;
};

struct PriorityItem {
  Vertex *vertex = nullptr;
  Vertex *prev_vertex = nullptr;
  double cost;
  double heuristic_cost;

  PriorityItem(Vertex *vertex, Vertex *prev_vertex, double cost, int goal_x,
               int goal_y, Options options);

  bool operator>(const PriorityItem &other) const;
};

class SThetaStar {
  private:
    nav2_costmap_2d::Costmap2D *costmap;
    std::string frame_id;
    rclcpp::Logger logger;
    Options options;
    bool has_prev;
    PathMsg prev_path;
    PoseStampedMsg prev_goal;
    double prev_cost;

    std::vector<Vertex> vertex_list;
    unsigned int width;
    unsigned int height;

  public:
    SThetaStar(nav2_costmap_2d::Costmap2D *costmap, std::string frame_id, Options options);
    PathMsg createPlan(const PoseStampedMsg &start, const PoseStampedMsg &goal, std::function<bool()> cancel_checker);
    void updateOptions(Options options);
    Options getOptions();

  private:
    void updateVertexList();
    PathMsg retracePath(Vertex vertex);
    double getDistance(double dx, double dy);
    double getCostmapCost(Coord coord) const;
    double getTraversalCost(Coord coord);
    bool isBlocked(Coord coord);
    bool hasLineOfSight(Coord initial, Coord end, double &cost);
    double calculatePathCost(PathMsg path);
    PoseStampedMsg moveGoal(PoseStampedMsg start, PoseStampedMsg goal);
};

#endif
