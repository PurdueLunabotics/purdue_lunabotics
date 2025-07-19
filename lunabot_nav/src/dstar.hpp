#ifndef DSTAR_H
#define DSTAR_H

#include <algorithm>
#include <iostream>
#include <climits>
#include <queue>
#include <tuple>
#include <set>
#include "math.h"

#define BUF_EXTRA 3 // add 3 extra -1 squares on each side of map grid when buffering
#define DBL_EPSILON 2.2204460492503131e-016

struct real_world_point {
  double x;
  double y;

  bool operator==(const real_world_point &other) const {
    return fabs(this->x - other.x) < DBL_EPSILON && fabs(this->y - other.y) < DBL_EPSILON;
  }
};

struct grid_point {
  int x;
  int y;

  bool operator==(const grid_point &other) const {
    return this->x == other.x && this->y == other.y;
  }

  bool operator!=(const grid_point &other) const {
    return this->x != other.x || this->y != other.y;
  }

  bool operator<(const grid_point &other) const { // need this to create a set
    return std::tie(this->x, this->y) < std::tie(other.x, other.y);
  }

  bool operator>(const grid_point &other) const { // need this to create a set
    return std::tie(this->x, this->y) > std::tie(other.x, other.y);
  }
};

struct node_key {
  double key_1;
  double key_2;

  bool operator<(const node_key &other) const {
    return std::tie(this->key_1, this->key_2) < std::tie(other.key_1, other.key_2);
  }

  bool operator>(const node_key &other) const {
    return std::tie(this->key_1, this->key_2) > std::tie(other.key_1, other.key_2);
  }
};

struct node {
  node_key key;
  grid_point point;

  bool operator>(const node &other) const { // needed for node queue
    return this->key > other.key;
  }

  bool operator==(const grid_point &other) const { // needed for node queue
    return this->point == other;
  }
};

struct node_value {
  double distance_g;
  double estimate_rhs;
};

class node_priority_queue : public std::priority_queue<node, std::vector<node>, std::greater<node>> {
public:
  bool remove_if_contains(const grid_point &value) {
    std::vector<node>::iterator it = std::find(this->c.begin(), this->c.end(), value);

    if (it == this->c.end()) {
      return false;
    }
    if (it == this->c.begin()) {
      // deque the top element
      this->pop();
    } else {
      // remove element and re-heap
      this->c.erase(it);
      std::make_heap(this->c.begin(), this->c.end(), this->comp);
    }
    return true;
  }
};

class Dstar {
public:
  Dstar(); // default constructor - functions should not be called until proper constructor run
  Dstar(real_world_point goal, real_world_point start, std::vector<std::vector<int>> init_map, double resolution, double x_offset, double y_offset, int occupancy_threshold = 50);
  void update_position(real_world_point new_pt);
  std::vector<real_world_point> find_path();
  std::vector<real_world_point> update_map(std::vector<std::vector<int>> new_map, double x_offset, double y_offset);

private:
  grid_point convert_to_grid(real_world_point pt);
  real_world_point convert_to_real(grid_point pt);
  node_key get_top_key();
  bool inside_map(grid_point point);
  void insert(grid_point point, node_key key);
  double hueristic(grid_point point);
  grid_point bfs_non_occupied(grid_point point);
  node_key calculate_key(grid_point point);
  double calculate_RHS(grid_point point);
  void update_point(grid_point point);
  std::vector<real_world_point> create_path_list();
  void buffer_map_for_goal();
  std::vector<real_world_point> update_replan(std::vector<std::vector<int>> prev_map, int new_map_offset_x, int new_map_offset_y,
                                              int buf_up, int buf_down, int buf_left, int buf_right);

  grid_point goal;
  real_world_point real_goal;

  grid_point current_point;
  grid_point prev_point;

  double resolution;

  // Real-world location of the 0,0 in the grid (not including buffer in the map)
  double x_offset;
  double y_offset;

  // The amount of buffer on any side of the map
  int buffer_offset_left = 0;
  int buffer_offset_up = 0;
  int buffer_offset_right = 0;
  int buffer_offset_down = 0;

  double km;
  double OCCUPANCY_THRESHOLD;

  node_priority_queue node_queue;
  std::vector<std::vector<int>> current_map;
  std::vector<std::vector<node_value>> node_values_list;
};

#endif