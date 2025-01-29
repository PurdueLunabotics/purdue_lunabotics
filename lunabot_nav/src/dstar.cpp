#include "dstar.hpp"

// TODO - search for safe goal

#define NUM_DIRECTIONS 4
grid_point cardinal_directions[NUM_DIRECTIONS] = {{0, -1}, {0, 1}, {-1, 0}, {1, 0}};

Dstar::Dstar() {};

Dstar::Dstar(real_world_point goal, real_world_point start, std::vector<std::vector<int>> init_map, double resolution, double x_offset, double y_offset, int occupancy_threshold) {
  this->km = 0.0; // Accumulation of distance from the last point(of changed map) to the current point

  // What number on the map corresponds to occupied
  this->OCCUPANCY_THRESHOLD = occupancy_threshold;

  // 2d array occupancy map : Occupancy probabilities from[0 to 100].Unknown is - 1.
  this->current_map = init_map;

  this->resolution = resolution;
  this->x_offset = x_offset;
  this->y_offset = y_offset;

  this->real_goal = goal;
  this->goal = convert_to_grid(goal);
  buffer_map_for_goal();
  this->goal = convert_to_grid(goal);

  node_values_list = std::vector<std::vector<node_value>>(current_map.size(), std::vector<node_value>(current_map[0].size(), {INT_MAX, INT_MAX}));
  this->node_values_list[this->goal.y][this->goal.x].estimate_rhs = 0;

  this->current_point = convert_to_grid(start);
  this->prev_point = convert_to_grid(start);

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
  return 0 <= point.y && point.y < current_map.size() && 0 <= point.x && point.x < current_map[0].size();
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

    if (inside_map(pt) && current_map[pt.y][pt.x] < OCCUPANCY_THRESHOLD) {
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
  node_value nv = node_values_list[point.y][point.x];
  double min_val = std::min(nv.distance_g, nv.estimate_rhs);

  double key1 = min_val + hueristic(point) + km;
  double key2 = min_val;

  return {key1, key2};
}

// Calculates the RHS(estimate value) of a given node by : first checking if it's an obstacle, then
// checking each surrounding node, calculating what the distance value should be based on those nodes,
// and taking the lowest value.
double Dstar::calculate_RHS(grid_point point) {
  if (current_map[point.y][point.x] > OCCUPANCY_THRESHOLD) {
    return INT_MAX;
  }

  double surrounding_values[NUM_DIRECTIONS + 1];

  // For each node(all 4 directions)
  for (int i = 0; i < NUM_DIRECTIONS; i++) {
    grid_point new_point = {point.x + cardinal_directions[i].x, point.y + cardinal_directions[i].y};

    // If the node is in bounds, and not an obstacle, add the distance value to the list
    if (inside_map(new_point) && current_map[new_point.y][new_point.x] < OCCUPANCY_THRESHOLD) {
      double g_val = node_values_list[new_point.y][new_point.x].distance_g;
      surrounding_values[i] = g_val + 1; // sqrt(2); TODO - handle non-cardinal directions - this should fix hugging obstacles
    } else {
      surrounding_values[i] = INT_MAX;
    }
  }
  surrounding_values[4] = INT_MAX;

  return *std::min_element(surrounding_values, surrounding_values + NUM_DIRECTIONS + 1);
}

// Updates a node's values by calculating its RHS (estimate value). It removes the node from the queue (based on old value) and
// replaces it on the queue if its values are locally inconsistent (if g != RHS)
void Dstar::update_point(grid_point point) {
  if (point != goal) {
    node_values_list[point.y][point.x].estimate_rhs = calculate_RHS(point);
  }

  node_queue.remove_if_contains(point);

  // Place it on the queue if not consistent
  node_value node = node_values_list[point.y][point.x];
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
  if (!inside_map(current_point) || current_map[current_point.y][current_point.x] > OCCUPANCY_THRESHOLD) {
    current_point = bfs_non_occupied(current_point);
  }

  // Loop until current node(start) is locally consistent(g == rhs) and its priority is lowest in the queue
  while (get_top_key() < calculate_key(current_point) ||
         node_values_list[current_point.y][current_point.x].distance_g != node_values_list[current_point.y][current_point.x].estimate_rhs) {

    node_key old_key = get_top_key();
    grid_point chosen_point = node_queue.top().point;
    node_value chosen_value = node_values_list[chosen_point.y][chosen_point.x];
    node_queue.pop();

    // If the priority of the node was incorrect, add back to the queue with the correct priority.
    if (old_key < calculate_key(chosen_point)) {
      insert(chosen_point, calculate_key(chosen_point));
    }

    // If g value is greater then rhs
    else if (chosen_value.distance_g > chosen_value.estimate_rhs) {
      // Lower the g value(the estimate is the more recent data)
      node_values_list[chosen_point.y][chosen_point.x].distance_g = chosen_value.estimate_rhs;

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
      node_values_list[chosen_point.y][chosen_point.x].distance_g = INT_MAX;

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
      if (inside_map(new_point) && current_map[new_point.y][new_point.x] < OCCUPANCY_THRESHOLD) {

        // We sort the path to take by two values - first the g value, then the euclidean distance to the goal.
        // This should pick, in event of a tie, the closer node to the goal.

        gvals[i].valid = true;
        gvals[i].g_val = (goal != new_point) ? node_values_list[new_point.y][new_point.x].distance_g + 1 : -1;
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

//  Add a buffer to the map so we can plan to the goal when it is outside the map.
// Saves the amount of buffer to allow for correct translations between real-world coords and grid coords.
void Dstar::buffer_map_for_goal() {
  int left_buf = std::max((0 - goal.x) + BUF_EXTRA, 0); // returns 0 if goal.x > 0, else -goal.x
  int right_buf = std::max((goal.x - (int)current_map[0].size()) + BUF_EXTRA, 0);
  int up_buf = std::max((0 - goal.y) + BUF_EXTRA, 0); // returns 0 if goal.x > 0, else -goal.x
  int down_buf = std::max((goal.y - (int)current_map.size()) + BUF_EXTRA, 0);

  int original_rows = current_map.size();
  int original_cols = current_map[0].size();

  // New dimensions for the buffered map
  int new_rows = original_rows + up_buf + down_buf;
  int new_cols = original_cols + left_buf + right_buf;
  buffer_offset_left = left_buf;
  buffer_offset_right = right_buf;
  buffer_offset_up = up_buf;
  buffer_offset_down = down_buf;

  // Create a new map filled with -1 (buffer)
  std::vector<std::vector<int>> buffered_map(new_rows, std::vector<int>(new_cols, -1));

  // Copy the original map into the new buffered map
  for (int y = 0; y < original_rows; y++) {
    for (int x = 0; x < original_cols; x++) {
      buffered_map[y + up_buf][x + left_buf] = current_map[y][x];
    }
  }

  // Replace the old map with the buffered map
  current_map = buffered_map;
}

// Update and replanning: Should trigger whenever there is a new map.
// Sets affected nodes to update, and calculates new g values (finds new path)
// This function runs when the map changes.

// Left offset and up offset are the amount of buffer added to the left and up sides of the map
// and are used to compare the correct parts of the map.

// Finally, it calculates the new path and returns the result
std::vector<real_world_point> Dstar::update_replan(std::vector<std::vector<int>> prev_map, int new_map_offset_x, int new_map_offset_y,
                                                   int buf_up, int buf_down, int buf_left, int buf_right) {
  // Add to the accumulation value the distance from the last point(of changed map) to the current point
  km += sqrt(pow(prev_point.x - current_point.x, 2) + pow(prev_point.y - current_point.y, 2));
  prev_point = current_point;

  for (int y = buf_up; y < prev_map.size() - buf_down; y++) {
    for (int x = buf_left; x < prev_map[0].size() - buf_right; x++) {
      if (current_map[y + new_map_offset_y - buf_up][x + new_map_offset_x - buf_left] != prev_map[y][x]) {
        update_point({x + new_map_offset_x - buf_left, y + new_map_offset_y - buf_up});
      }
    }
  }

  return find_path();
}

// Updates the map with new grid whenever map is changed.The node values list is expanded if needed to match the new size of the map.
// The map is updated with the new given map, and buffer is added so that we never have to shrink the map / node values.

// After the new map is built, we call update / replan, updating the needed node values, which later calculates the path and returns the new path
std::vector<real_world_point> Dstar::update_map(std::vector<std::vector<int>> new_map, double x_offset, double y_offset) {
  double prev_x_offset = this->x_offset;
  double prev_y_offset = this->y_offset;

  // Find the size of the data in the old map
  std::vector<std::vector<int>> prev_map = current_map;
  int prev_map_unbuf_width = prev_map[0].size() - buffer_offset_left - buffer_offset_right;
  int prev_map_unbuf_height = prev_map.size() - buffer_offset_up - buffer_offset_down;

  // Find how many new columns and rows are present (>= 0)
  int map_new_cols_left = (prev_x_offset - x_offset) / resolution;
  int map_new_cols_right = new_map[0].size() - prev_map_unbuf_width - map_new_cols_left;
  int map_new_rows_up = (prev_y_offset - y_offset) / resolution;
  int map_new_rows_down = new_map.size() - prev_map_unbuf_height - map_new_rows_up;

  // Figure out how much buffer to add to the map, don't need to add buffer if the new map data covers it
  int map_buf_cols_left = std::max(buffer_offset_left - map_new_cols_left, 0);
  int map_buf_cols_right = std::max(buffer_offset_right - map_new_cols_right, 0);
  int map_buf_rows_up = std::max(buffer_offset_up - map_new_rows_up, 0);
  int map_buf_rows_down = std::max(buffer_offset_down - map_new_rows_down, 0);

  int map_original_rows = new_map.size();
  int map_original_cols = new_map[0].size();

  // New dimensions for the buffered map
  int map_new_rows = map_original_rows + map_buf_rows_up + map_buf_rows_down;
  int map_new_cols = map_original_cols + map_buf_cols_left + map_buf_cols_right;

  // Create a new map filled with -1 (buffer)
  current_map = std::vector<std::vector<int>>(map_new_rows, std::vector<int>(map_new_cols, -1));

  // Copy the original map into the new buffered map
  for (int y = 0; y < map_original_rows; y++) {
    for (int x = 0; x < map_original_cols; x++) {
      current_map[y + map_buf_rows_up][x + map_buf_cols_left] = new_map[y][x];
    }
  }

  // Find how many columns and rows we'll have to append to node values (the number of new columns/rows, unless already present in the buffer)
  int nv_buf_cols_left = std::max(map_new_cols_left - buffer_offset_left, 0);
  int nv_buf_cols_right = std::max(map_new_cols_right - buffer_offset_right, 0);
  int nv_buf_rows_up = std::max(map_new_rows_up - buffer_offset_up, 0);
  int nv_buf_rows_down = std::max(map_new_rows_down - buffer_offset_down, 0);

  int nv_original_rows = node_values_list.size();
  int nv_original_cols = node_values_list[0].size();

  // New dimensions for the buffered map
  int nv_new_rows = nv_original_rows + nv_buf_rows_up + nv_buf_rows_down;
  int nv_new_cols = nv_original_cols + nv_buf_cols_left + nv_buf_cols_right;

  // Create a new map filled with {INT_MAX, INT_MAX}
  std::vector<std::vector<node_value>> prev_nv = node_values_list;
  node_values_list = std::vector<std::vector<node_value>>(nv_new_rows, std::vector<node_value>(nv_new_cols, {INT_MAX, INT_MAX}));
  // Copy the original node values into the new node_values
  for (int y = 0; y < nv_original_rows; y++) {
    for (int x = 0; x < nv_original_cols; x++) {
      node_values_list[y + nv_buf_rows_up][x + nv_buf_cols_left] = prev_nv[y][x];
    }
  }

  int old_buf_up = buffer_offset_up;
  int old_buf_down = buffer_offset_down;
  int old_buf_left = buffer_offset_left;
  int old_buf_right = buffer_offset_right;

  buffer_offset_left = map_buf_cols_left;
  buffer_offset_right = map_buf_cols_right;
  buffer_offset_up = map_buf_rows_up;
  buffer_offset_down = map_buf_rows_down;
  this->x_offset = x_offset;
  this->y_offset = y_offset;

  // Change the value of the current node - these values(new rows / cols of node values) represent how much the physical size of the map has changed
  current_point.x += nv_buf_cols_left;
  current_point.y += nv_buf_rows_up;

  // change goal - as the size / buffer of the map has changed, the goal should be reconverted
  goal = convert_to_grid(real_goal);

  return update_replan(prev_map, map_new_cols_left, map_new_rows_up, old_buf_up, old_buf_down, old_buf_left, old_buf_right);
}