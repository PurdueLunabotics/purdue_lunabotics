#ifndef MAP_H
#define MAP_H

#include <assert.h>
#include <cassert>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <vector>

#include <iostream>

class Map {
private:
  int width_;
  int height_;
  std::vector<int8_t> grid_;
  std::vector<int> origin_;
  double resolution_;
  int occupied_val_;

public:
  Map() : width_{-1}, height_{-1}, resolution_{-1} {
    ros::param::get("occupied_val", occupied_val_);
    std::cout << "Occupied val: " << occupied_val_ << std::endl;
  }
  Map(const nav_msgs::OccupancyGrid &map) { update_map(map); }

  bool occupied_at_pos(std::vector<double> state) {
    assert(resolution_ != -1);
    assert(state.size() == 2);

    // Map offset
    for (int i = 0; i < state.size(); i++) {
      state[i] -= origin_[i];
    }
    // Convert to grid coords
    for (int i = 0; i < state.size(); i++) {
      state[i] /= resolution_;
    }
    for (int i = 0; i < state.size(); i++) {
      state[i] = round(state[i]);
    }
    std::vector<int> uv_ind(state.begin(), state.end());
    int flat_ind = height_ * uv_ind[1] + uv_ind[0];
    if (flat_ind < 0 || flat_ind >= grid_.size())
      return true;
    return grid_[flat_ind] >= 10;
  }

  void update_map(const nav_msgs::OccupancyGrid &map) {
    resolution_ = map.info.resolution;
    width_ = map.info.width;
    height_ = map.info.height;
    origin_.push_back(map.info.origin.position.x);
    origin_.push_back(map.info.origin.position.y);
    grid_ = map.data;
  }
};

#endif