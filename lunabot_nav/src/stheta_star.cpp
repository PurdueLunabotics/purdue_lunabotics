#include "stheta_star.hpp"
#include <cmath>
#include <limits>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <angles/angles.h>

const int LETHAL_COST = 252;

Coord::Coord(int x, int y) {
  this->x = x;
  this->y = y;
}

Coord::Coord() {}

double distSquared(Coord a, Coord b) {
  return std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2);
}

double calculateAlphaCost(Coord current, Coord grandparent, Coord goal) {
  return std::acos(
      (distSquared(grandparent, current) + distSquared(grandparent, goal) - distSquared(current, goal)) /
      (2 * std::sqrt(distSquared(grandparent, current)) * std::sqrt(distSquared(grandparent, goal))));
}

PriorityItem::PriorityItem(Vertex *vertex, Vertex *prev_vertex, double cost, int goal_x,
             int goal_y, Options options) {
  double alpha_cost = 0;
  if (prev_vertex && (prev_vertex->prev_coord.x != -1 || prev_vertex->prev_coord.y != -1)) {
    alpha_cost = calculateAlphaCost(vertex->coord, prev_vertex->prev_coord, Coord(goal_x, goal_y));
  }

  this->vertex = vertex;
  this->cost = cost + options.node_cost + alpha_cost * options.turning_cost;
  this->prev_vertex = prev_vertex;
  this->heuristic_cost =
      std::sqrt(std::pow((double)goal_x - (double)vertex->coord.x, 2.) +
                std::pow((double)goal_y - (double)vertex->coord.y, 2.));
}

bool PriorityItem::operator>(const PriorityItem &other) const {
  return cost + heuristic_cost > other.cost + other.heuristic_cost;
}

SThetaStar::SThetaStar(nav2_costmap_2d::Costmap2D *costmap, std::string frame_id, Options options) : logger(rclcpp::get_logger("SThetaStar")) {
  this->costmap = costmap;
  this->frame_id = frame_id;
  this->options = options;
  this->has_prev = false;
  updateVertexList();
}

void SThetaStar::updateOptions(Options options) {
  RCLCPP_INFO(logger, "set options");
  this->has_prev = false;
  this->options = options;
}

Options SThetaStar::getOptions() {
  return options;
}

void SThetaStar::updateVertexList() {
  width = costmap->getSizeInCellsX();
  height = costmap->getSizeInCellsY();

  vertex_list = std::vector<Vertex>(width * height);
  for (unsigned int y = 0; y < height; y++) {
    for (unsigned int x = 0; x < width; x++) {
      vertex_list[x + y * width].coord.x = x;
      vertex_list[x + y * width].coord.y = y;
    }
  }
}

PathMsg SThetaStar::retracePath(Vertex vertex) {
  PathMsg path;
  path.header.stamp = rclcpp::Time();
  path.header.frame_id = frame_id;

  while (true) {
    PoseStampedMsg pose;

    pose.header.stamp = rclcpp::Time();
    pose.header.frame_id = frame_id;
    costmap->mapToWorld((unsigned int)vertex.coord.x,
                        (unsigned int)vertex.coord.y, pose.pose.position.x,
                        pose.pose.position.y);
    pose.pose.position.z = 0;

    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    path.poses.push_back(pose);
    if (vertex.prev_coord.x == -1 && vertex.prev_coord.y == -1) {
      break;
    }
    vertex = vertex_list[vertex.prev_coord.x + vertex.prev_coord.y * width];
  }
  std::reverse(path.poses.begin(), path.poses.end());
  return path;
}

PathMsg SThetaStar::createPlan(const PoseStampedMsg &start,
                               const PoseStampedMsg &originalGoal) {
  if (costmap->getSizeInCellsX() != width ||
      costmap->getSizeInCellsY() != height) {
    updateVertexList();
  }

  auto goal = moveGoal(start, originalGoal);

  if (has_prev && prev_goal == goal && !prev_path.poses.empty()) {
    prev_path.poses[0] = start;
    double new_cost = calculatePathCost(prev_path);
    if (std::abs(new_cost - prev_cost) / prev_cost < options.remapping_cost_change_percent) {
      if (new_cost < prev_cost) {
        prev_cost = new_cost;
      }
      RCLCPP_DEBUG(logger, "Sending modified old path");
      return prev_path;
    }
  }

  for (unsigned int i = 0; i < width * height; i++) {
    vertex_list[i].visited = false;
  }

  std::priority_queue<PriorityItem, std::vector<PriorityItem>,
                      std::greater<PriorityItem>>
      queue;
  int goal_x, goal_y;
  costmap->worldToMapNoBounds(goal.pose.position.x, goal.pose.position.y,
                              goal_x, goal_y);

  int initial_x, initial_y;
  costmap->worldToMapNoBounds(start.pose.position.x, start.pose.position.y,
                              initial_x, initial_y);
  Vertex *initial = &vertex_list[initial_x + initial_y * width];
  initial->prev_coord.x = -1;
  initial->prev_coord.y = -1;

  queue.push(
      PriorityItem(initial, nullptr, 0, goal_x, goal_y, options));

  while (!queue.empty()) {
    PriorityItem item = queue.top();
    queue.pop();
    if (item.vertex->visited) {
      continue;
    }

    item.vertex->cost = item.cost;
    item.vertex->visited = true;
    if (item.prev_vertex) {
      item.vertex->prev_coord = item.prev_vertex->coord;
    } else {
      item.vertex->prev_coord.x = -1;
      item.vertex->prev_coord.y = -1;
    }

    if (item.vertex->coord.x == goal_x && item.vertex->coord.y == goal_y) {
      auto path = retracePath(*item.vertex);
      path.poses.back() = goal;
      this->prev_path = path;
      this->prev_cost = item.vertex->cost;
      this->prev_goal = goal;
      this->has_prev = true;
      return path;
    }

    int x_offsets[] = {-1, 1, 0, 0};
    int y_offsets[] = {0, 0, -1, 1};

    for (int i = 0; i < 4; i++) {
      int new_x = item.vertex->coord.x + x_offsets[i];
      int new_y = item.vertex->coord.y + y_offsets[i];

      if (isBlocked(Coord(new_x, new_y)) ||
          vertex_list[new_x + new_y * width].visited) {
        continue;
      }

      if (item.prev_vertex) {
        Coord prev_coord = item.prev_vertex->coord;
        double los_cost =
            getDistance(prev_coord.x - new_x, prev_coord.y - new_y) * options.driving_cost;
        if (hasLineOfSight(prev_coord, Coord(new_x, new_y), los_cost)) {
          PriorityItem queue_item(
              &vertex_list[new_x + new_y * width], item.prev_vertex,
              vertex_list[prev_coord.x + prev_coord.y * width].cost +
                  los_cost, goal_x, goal_y, options);
          queue.push(queue_item);
        }
      }

      PriorityItem queue_item(
          &vertex_list[new_x + new_y * width], item.vertex,
          item.cost + options.driving_cost + getTraversalCost(Coord(new_x, new_y)), goal_x,
          goal_y, options);
      queue.push(queue_item);
    }
  }

  return PathMsg();
}

double SThetaStar::getDistance(double dx, double dy) {
  return std::sqrt(std::pow(dx, 2.0) + std::pow(dy, 2.0));
}

double SThetaStar::getCostmapCost(Coord coord) const {
  return 26 + 0.9 * costmap->getCost(coord.x, coord.y);
}

double SThetaStar::getTraversalCost(Coord coord) {
  double curr_cost = getCostmapCost(coord);
  return options.traversal_cost * std::pow(curr_cost / (double)LETHAL_COST, options.costmap_exponential);
}

bool SThetaStar::isBlocked(Coord coord) {
  return coord.x < 0 || coord.y < 0 || (unsigned int)coord.x >= width ||
         (unsigned int)coord.y >= height ||
         costmap->getCost(coord.x, coord.y) >= LETHAL_COST;
}

bool SThetaStar::hasLineOfSight(Coord initial, Coord end, double &cost) {
  Coord current = initial;
  int distance_x = abs(initial.x - end.x);
  int distance_y = abs(initial.y - end.y);

  if (distance_x == 0 && distance_y == 0) {
    return true;
  }

  int divisor = std::gcd(distance_x, distance_y);
  int dx = (end.x - initial.x) / divisor;
  int dy = (end.y - initial.y) / divisor;

  int x_increment = 0;
  if (distance_x != 0) {
    x_increment = (end.x - current.x) / distance_x;
  }
  int y_increment = 0;
  if (distance_y != 0) {
    y_increment = (end.y - current.y) / distance_y;
  }

  if (distance_x > distance_y) {
    while (current.x != end.x) {
      current.x += x_increment;
      int target_y = dy * (current.x - initial.x) / dx + initial.y;
      if (target_y != current.y) {
        current.y += y_increment;
      }

      if (isBlocked(current)) {
        return false;
      }

      cost += getTraversalCost(current);
    }
  } else {
    while (current.y != end.y) {
      current.y += y_increment;
      int target_x = dx * (current.y - initial.y) / dy + initial.x;
      if (target_x != current.x) {
        current.x += x_increment;
      }

      if (isBlocked(current)) {
        return false;
      }

      cost += getTraversalCost(current);
    }
  }

  return true;
}

double SThetaStar::calculatePathCost(PathMsg path) {
  double cost = options.node_cost;
  unsigned int x, y;

  Coord goal;
  costmap->worldToMap(path.poses.end()->pose.position.x, path.poses.end()->pose.position.y, x, y);
  goal.x = x;
  goal.y = y;

  for (int i = 0; i < (int) path.poses.size() - 1; i++) {
    Coord start, end;

    costmap->worldToMap(path.poses[i].pose.position.x, path.poses[i].pose.position.y, x, y);
    start.x = x;
    start.y = y;
    costmap->worldToMap(path.poses[i + 1].pose.position.x, path.poses[i + 1].pose.position.y, x, y);
    end.x = x;
    end.y = y;

    if (start.x == end.x && start.y == end.y) {
      continue;
    }

    double traversal_cost = 0;

    if (!hasLineOfSight(start, end, traversal_cost)) {
      return std::numeric_limits<double>::infinity();
    }

    int dx = end.x - start.x;
    int dy = end.y - start.y;

    double alpha_cost = 0;
    if (i > 0) {
      Coord grandparent;
      costmap->worldToMap(path.poses[i - 1].pose.position.x, path.poses[i - 1].pose.position.y, x, y);
      grandparent.x = x;
      grandparent.y = y;
      alpha_cost = calculateAlphaCost(end, grandparent, goal);
    }

    cost +=
        traversal_cost +
        getDistance(dx, dy) * options.driving_cost +
        alpha_cost * options.driving_cost +
        options.node_cost;
  }

  return cost;
}

PoseStampedMsg SThetaStar::moveGoal(PoseStampedMsg start, PoseStampedMsg goal) {
    PoseStampedMsg newGoal;
    int x, y;
    this->costmap->worldToMapEnforceBounds(goal.pose.position.x, goal.pose.position.y, x, y);
    Coord initial = {x, y};
    this->costmap->worldToMapEnforceBounds(start.pose.position.x, start.pose.position.y, x, y);
    Coord end = {x, y};

    if (initial.x == end.x && initial.y == end.y) {
      return goal;
    }

    Coord current = initial;
    int distance_x = abs(initial.x - end.x);
    int distance_y = abs(initial.y - end.y);

    int divisor = std::gcd(distance_x, distance_y);
    int dx = (end.x - initial.x) / divisor;
    int dy = (end.y - initial.y) / divisor;

    int x_increment = 0;
    if (distance_x != 0) {
      x_increment = (end.x - current.x) / distance_x;
    }
    int y_increment = 0;
    if (distance_y != 0) {
      y_increment = (end.y - current.y) / distance_y;
    }

    if (distance_x > distance_y) {
      while (isBlocked(current) && getDistance(current.x - initial.x, current.y - initial.y) * costmap->getResolution() < options.max_goal_adjustment_meters) {
        current.x += x_increment;
        int target_y = dy * (current.x - initial.x) / dx + initial.y;
        if (target_y != current.y) {
          current.y += y_increment;
        }
      }
    } else {
      while (isBlocked(current) && getDistance(current.x - initial.x, current.y - initial.y) * costmap->getResolution() < options.max_goal_adjustment_meters) {
        current.y += y_increment;
        int target_x = dx * (current.y - initial.y) / dy + initial.x;
        if (target_x != current.x) {
          current.x += x_increment;
        }
      }
    }

    newGoal.header = goal.header;
    newGoal.pose.orientation = goal.pose.orientation;
    newGoal.pose.position = goal.pose.position;

    for (unsigned int i = 0; i < width * height; i++) {
      vertex_list[i].visited = false;
    }


    std::queue<Coord> queue;
    queue.push(current);
    while (!queue.empty()) {
      if (!isBlocked(queue.front())) {
        this->costmap->mapToWorld(queue.front().x, queue.front().y, newGoal.pose.position.x, newGoal.pose.position.y);
        break;
      }

      Coord offsets[4] = {{-1, 0}, {1, 0}, {0, 1}, {0, -1}};
      for (auto offset : offsets) {
        Coord new_coord = {queue.front().x + offset.x, queue.front().y + offset.y};
        if (new_coord.x < 0 || new_coord.x >= (int) width || new_coord.y < 0 || new_coord.y >= (int) height) {
          continue;
        }
        if (!vertex_list[new_coord.x + new_coord.y * width].visited) {
          queue.push(new_coord);
          vertex_list[new_coord.x + new_coord.y * width].visited = true;
        }
      }

      queue.pop();
    }

    return newGoal;
}
