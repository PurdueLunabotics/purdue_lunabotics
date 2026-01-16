#include "stheta_star.hpp"
#include <rclcpp/logging.hpp>

const int LETHAL_COST = 252;

Coord::Coord(int x, int y) {
  this->x = x;
  this->y = y;
}

Coord::Coord() {}

PriorityItem::PriorityItem(Vertex *vertex, Coord prev_coord, double cost, int goal_x,
             int goal_y, double prev_dir, Options options) {
  double current_dir = std::atan2(prev_coord.y - vertex->coord.y,
                                  prev_coord.x - vertex->coord.x);

  this->vertex = vertex;
  this->cost = cost + options.node_cost;
  this->prev_coord = prev_coord;
  this->heuristic_cost =
      std::sqrt(std::pow((double)goal_x - (double)vertex->coord.x, 2.) +
                std::pow((double)goal_y - (double)vertex->coord.y, 2.)) +
      std::abs(current_dir - prev_dir) * options.turning_cost;
}

bool PriorityItem::operator>(const PriorityItem &other) const {
  return cost + heuristic_cost > other.cost + other.heuristic_cost;
}

SThetaStar::SThetaStar(nav2_costmap_2d::Costmap2D *costmap, std::string frame_id, Options options) : logger(rclcpp::get_logger("SThetaStar")) {
  this->costmap = costmap;
  this->frame_id = frame_id;
  this->options = options;
  updateVertexList();
}

void SThetaStar::updateOptions(Options options) {
  RCLCPP_INFO(logger, "set options");
  this->options = options;
}

Options SThetaStar::getOptions() {
  return options;
}

void SThetaStar::updateVertexList() {
  width = costmap->getSizeInCellsX();
  height = costmap->getSizeInCellsY();

  vertex_list = new Vertex[width * height];
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
                               const PoseStampedMsg &goal) {
  if (costmap->getSizeInCellsX() != width ||
      costmap->getSizeInCellsY() != height) {
    delete[] vertex_list;
    updateVertexList();
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
      PriorityItem(initial, Coord(-1, -1), 0, goal_x, goal_y,
                   0, options));  // FIXME: make this actually use the robot rotation

  while (!queue.empty()) {
    PriorityItem item = queue.top();
    queue.pop();
    if (item.vertex->visited) {
      continue;
    }

    item.vertex->cost = item.cost;
    item.vertex->visited = true;
    item.vertex->prev_coord = item.prev_coord;

    if (item.vertex->coord.x == goal_x && item.vertex->coord.y == goal_y) {
      return retracePath(*item.vertex);
    }

    double current_dir =
        std::atan2(item.vertex->prev_coord.y - item.vertex->coord.y,
                   item.vertex->prev_coord.x - item.vertex->coord.x);

    int x_offsets[] = {-1, 1, 0, 0};
    int y_offsets[] = {0, 0, -1, 1};

    for (int i = 0; i < 4; i++) {
      int new_x = item.vertex->coord.x + x_offsets[i];
      int new_y = item.vertex->coord.y + y_offsets[i];

      if (isBlocked(Coord(new_x, new_y)) ||
          vertex_list[new_x + new_y * width].visited) {
        continue;
      }

      double los_cost =
          getDistance(item.prev_coord.x - new_x, item.prev_coord.y - new_y) * options.driving_cost;
      if (hasLineOfSight(item.prev_coord, Coord(new_x, new_y), los_cost)) {
        PriorityItem queue_item(
            &vertex_list[new_x + new_y * width], item.prev_coord,
            vertex_list[item.prev_coord.x + item.prev_coord.y * width].cost +
                los_cost,
            goal_x, goal_y, current_dir, options);
        queue.push(queue_item);
      }

      PriorityItem queue_item(
          &vertex_list[new_x + new_y * width], item.vertex->coord,
          item.cost + options.driving_cost + getTraversalCost(Coord(new_x, new_y)), goal_x,
          goal_y, current_dir, options);
      queue.push(queue_item);
    }
  }

  return PathMsg();
}

double SThetaStar::getDistance(double dx, double dy) {
  return std::sqrt(std::pow(dx, 2.0) + std::pow(dy, 2.0));
}

double SThetaStar::getCost(Coord coord) const {
  return 26 + 0.9 * costmap->getCost(coord.x, coord.y);
}

double SThetaStar::getTraversalCost(Coord coord) {
  double curr_cost = getCost(coord);
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
