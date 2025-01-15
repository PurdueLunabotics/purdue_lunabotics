#include "dstar.hpp"

#define NUM_DIRECTIONS 4
grid_point cardinal_directions[NUM_DIRECTIONS] = {{0, -1}, {0, 1}, {-1, 0}, {1, 0}};

Dstar::Dstar() {};

Dstar::Dstar(real_world_point goal, real_world_point start, std::vector<std::vector<int>> init_map, double resolution, double x_offset, double y_offset, int occupancy_threshold) {
  // TODO - map resize

  this->km = 0.0; // Accumulation of distance from the last point(of changed map) to the current point

  // What number on the map corresponds to occupied
  this->OCCUPANCY_THRESHOLD = occupancy_threshold;

  node_values_list = std::vector<std::vector<node_value>>(init_map.size(), std::vector<node_value>(init_map[0].size()));

  // 2d Array of nodes that corresponds to the map : each value is[Distance(g), Estimate(rhs)]
  for (int x = 0; x < node_values_list.size(); x++) {
    for (int y = 0; y < node_values_list[y].size(); y++) {
      node_values_list[x][y].distance_g = INT_MAX;
      node_values_list[x][y].estimate_rhs = INT_MAX;
    }
  }

  // 2d array occupancy map : Occupancy probabilities from[0 to 100].Unknown is - 1.
  this->current_map = init_map;

  this->resolution = resolution;
  this->x_offset = x_offset;
  this->y_offset = y_offset;

  this->goal = convert_to_grid(goal);
  this->start = convert_to_grid(start);

  this->node_values_list[this->goal.x][this->goal.y].estimate_rhs = 0;

  this->current_point = this->start;
  this->prev_point = this->start;

  // Insert the goal node into the priority queue
  this->insert(this->goal, this->calculate_key(this->goal));
}

// When receiving a new position from the node, change it to grid coords and update the current node
void Dstar::update_position(real_world_point new_pt) {
  current_point = convert_to_grid(new_pt);
}

// Convert a real world(x y) position to grid coordinates.Grid offset should be in the same frame as position
grid_point Dstar::convert_to_grid(real_world_point pt) {
  return {
      (int)((pt.x - x_offset) / resolution + 0.5) + buffer_offset_left,
      (int)((pt.y - y_offset) / resolution + 0.5) + buffer_offset_up};
}

// Convert a grid position to real world (x y) coordinates. Grid offset should be in the same frame as position
real_world_point Dstar::convert_to_real(grid_point pt) {
  return {
      (pt.x + 0.5 - buffer_offset_left) * resolution + x_offset,
      (pt.y + 0.5 - buffer_offset_up) * resolution + y_offset};
}

// returns true if point is inside the map boundaries
bool Dstar::inside_map(grid_point point) {
  return 0 <= point.x && point.x < current_map.size() && 0 <= point.y && point.y < current_map[0].size();
}

// returns the lowest priority in the queue
node_key Dstar::get_top_key() {
  if (node_queue.empty()) {
    return {(double)INT_MAX, (double)INT_MAX};
  } else {
    return node_queue.top().key;
  }
}

// inserts a node into the queue
void Dstar::insert(grid_point point, node_key key) {
  node newNode = {key, point};
  node_queue.push(newNode);
}

// Calculates the hueristic used for the priority of a node based on its distance to the goal
double Dstar::hueristic(grid_point point) {
  return sqrt(pow(point.x - current_point.x, 2) + pow(point.y - current_point.y, 2));
}

// Searches for the nearest non-occupied node closest to the given node. Used for finding a path if the robot is stuck on top of an obstacle.
grid_point Dstar::bfs_non_occupied(grid_point point) {
  std::queue<grid_point> pt_queue;
  std::set<grid_point> visited_pts;
  pt_queue.push(point);

  while (!pt_queue.empty()) {
    grid_point pt = pt_queue.front();
    pt_queue.pop();

    if (visited_pts.count(pt)) { // 0 or 1
      continue;
    }

    if (inside_map(pt) && current_map[pt.x][pt.y] < OCCUPANCY_THRESHOLD) {
      return pt;
    }

    for (int i = 0; i < NUM_DIRECTIONS; i++) {
      grid_point new_point = {pt.x + cardinal_directions[i].x, pt.y + cardinal_directions[i].y};

      // If the node is in bounds, and not an obstacle, add the distance value to the list
      if (inside_map(new_point)) {
        pt_queue.push(new_point);
      }
    }

    visited_pts.insert(pt);
  }
  std::cout << "Dstar: Error in search for non-occupied node" << std::endl;
  return point;
}

// Calculates the priority of a node based on its g and rhs values
node_key Dstar::calculate_key(grid_point point) {
  node_value nv = node_values_list[point.x][point.y];
  double min_val = std::min(nv.distance_g, nv.estimate_rhs);

  double key1 = min_val + hueristic(point) + km;
  double key2 = min_val;

  return {key1, key2};
}

// Calculates the RHS(estimate value) of a given node by : first checking if it's an obstacle, then
// checking each surrounding node, calculating what the distance value should be based on those nodes,
// and taking the lowest value.
double Dstar::calculate_RHS(grid_point point) {
  if (current_map[point.x][point.y] > OCCUPANCY_THRESHOLD) {
    return INT_MAX;
  }

  double surrounding_values[NUM_DIRECTIONS];

  // For each node(all 4 directions)
  for (int i = 0; i < NUM_DIRECTIONS; i++) {
    grid_point new_point = {point.x + cardinal_directions[i].x, point.y + cardinal_directions[i].y};

    // If the node is in bounds, and not an obstacle, add the distance value to the list
    if (inside_map(new_point) && current_map[new_point.x][new_point.y] < OCCUPANCY_THRESHOLD) {
      double g_val = node_values_list[new_point.x][new_point.y].distance_g;
      surrounding_values[i] = g_val + 1; // sqrt(2); TODO - handle non-cardinal directions
    } else {
      surrounding_values[i] = INT_MAX;
    }
  }

  return *std::min_element(surrounding_values, surrounding_values + 4);
}

// Updates a node's values by calculating its RHS (estimate value). It removes the node from the queue (based on old value) and
// replaces it on the queue if its values are locally inconsistent (if g != RHS)
void Dstar::update_point(grid_point point) {
  if (point != goal) {
    node_values_list[point.x][point.y].estimate_rhs = calculate_RHS(point);
  }

  node_queue.remove_if_contains(point);

  int iterations = node_queue.size();
  for (int i = 0; i < iterations; i++) {
    node c_node = node_queue.top();
    node_queue.pop();
    if (c_node.point != point) {
      node_queue.push(c_node);
    }
  }

  // Place it on the queue if not consistent
  node_value node = node_values_list[point.x][point.y];
  if (node.distance_g != node.estimate_rhs) {
    insert(point, calculate_key(point));
  }
}

// Find_Path calculates the path for DStar by looping until the current node is locally consistent (g value = rhs) and the priority of the current node is the lowest in the queue.
// It picks the lowest priority node, checks whether its priority is correct, then updates its g value (distance). If the g value is higher then the estimate, it lowers the g value to the estimate.
// If the g value is lower then the estimate, it sets it for correction by setting the g value to infinity. It then marks all surrounding nodes to be checked.
//
// Through this process, all nodes on the grid have their g value calculated correctly so the path can be found.
//
// Once finished, returns the calculated path from the start to the goal.
std::vector<real_world_point> Dstar::find_path() {
  std::cout << "Dstar: Finding path" << std::endl;

  // If the start is out of the map, or is an obstacle, search for the closest non - occupied node
  if (!inside_map(current_point) || current_map[current_point.x][current_point.y] > OCCUPANCY_THRESHOLD) {
    current_point = bfs_non_occupied(current_point);
  }

  // Loop until current node(start) is locally consistent(g == rhs) and its priority is lowest in the queue
  while (get_top_key() < calculate_key(current_point) ||
         node_values_list[current_point.x][current_point.y].distance_g != node_values_list[current_point.x][current_point.y].estimate_rhs) {

    node_key old_key = get_top_key();
    grid_point chosen_point = node_queue.top().point;
    node_value chosen_value = node_values_list[chosen_point.x][chosen_point.y];
    node_queue.pop();

    // If the priority of the node was incorrect, add back to the queue with the correct priority.
    if (old_key < calculate_key(chosen_point)) {
      insert(chosen_point, calculate_key(chosen_point));
    }

    // If g value is greater then rhs
    else if (chosen_value.distance_g > chosen_value.estimate_rhs) {
      // Lower the g value(the estimate is the more recent data)
      node_values_list[chosen_point.x][chosen_point.y].distance_g = chosen_value.estimate_rhs;

      // update all surrounding nodes
      for (int i = 0; i < NUM_DIRECTIONS; i++) {
        grid_point new_point = {chosen_point.x + cardinal_directions[i].x, chosen_point.y + cardinal_directions[i].y};
        if (inside_map(new_point)) {
          update_point(new_point);
        }
      }
    }

    // G is lower then rhs
    else {
      // Set g to infinity(mark it for replanning)
      node_values_list[chosen_point.x][chosen_point.y].distance_g = INT_MAX;

      // Update this node
      update_point(chosen_point);

      // update all surrounding nodes
      for (int i = 0; i < NUM_DIRECTIONS; i++) {
        grid_point new_point = {chosen_point.x + cardinal_directions[i].x, chosen_point.y + cardinal_directions[i].y};
        if (inside_map(new_point)) {
          update_point(new_point);
        }
      }
    }

    if (node_queue.size() == 0) {
      std::cout << "Dstar: No path found (map processing failed)" << std::endl;
      break;
    }
  }

  return create_path_list();
}

// Create path: This creates a temporary node at the start, and picks the lowest g value (lowest distance to goal) as the next step on the path, adds it to the list, and
// repeats until the goal is reached. All of these points in order are the path.
std::vector<real_world_point> Dstar::create_path_list() {
  std::cout << "Dstar: Generating path" << std::endl;

  std::vector<real_world_point> path_list;

  // if start = goal, there is no path
  if (current_point == goal) {
    std::cout << "Dstar: No path (start = goal)" << std::endl;
    return path_list;
  }

  grid_point path_point = current_point;

  // Until robot reaches self.goal
  while (path_point != goal) {
    struct {
      bool valid;
      double g_val;
      double heuristic;
      grid_point pt;
    } gvals[NUM_DIRECTIONS];

    for (int i = 0; i < NUM_DIRECTIONS; i++) {
      grid_point new_point = {path_point.x + cardinal_directions[i].x, path_point.y + cardinal_directions[i].y};

      // if in bounds, and not an obstacle, add the g value to the list
      if (inside_map(new_point) && current_map[new_point.x][new_point.y] < OCCUPANCY_THRESHOLD) {

        // We sort the path to take by two values - first the g value, then the euclidean distance to the goal.
        // This should pick, in event of a tie, the closer node to the goal.

        gvals[i].valid = true;
        gvals[i].g_val = (goal != new_point) ? node_values_list[new_point.x][new_point.y].distance_g + 1 : -1;
        gvals[i].heuristic = sqrt(pow(goal.x - new_point.x, 2) + pow(goal.y - new_point.y, 2));
        gvals[i].pt = new_point;
      } else {
        gvals[i].valid = false;
      }
    }

    bool found = false;
    for (int i = 0; i < NUM_DIRECTIONS; i++) {
      found = gvals[i].valid || found;
    }

    if (!found) {
      std::cout << "Dstar: No path (couldn't create a complete path list)" << std::endl;
      return std::vector<real_world_point>();
    }

    double best_g_val = INT_MAX;
    double best_tie_break = INT_MAX;
    for (int i = 0; i < NUM_DIRECTIONS; i++) {
      if (gvals[i].valid && gvals[i].g_val <= best_g_val && gvals[i].heuristic < best_tie_break) {
        path_point = gvals[i].pt;
        best_g_val = gvals[i].g_val;
        best_tie_break = gvals[i].heuristic;
      }
    }

    real_world_point rwp = convert_to_real(path_point);

    for (real_world_point pt : path_list) {
      if (pt == rwp) {
        // Doubling back- no more path
        std::cout << "Dstar: No path (path list incomplete (would double back))" << std::endl;
        return std::vector<real_world_point>();
      }
    }
    path_list.push_back(rwp);
  }

  return path_list;
}

// TODO update_replan
// TODO buffer_map_for_goal
// TODO update_map