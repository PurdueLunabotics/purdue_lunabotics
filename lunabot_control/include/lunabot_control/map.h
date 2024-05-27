#ifndef MAP_H
#define MAP_H

#include <assert.h>
#include <cassert>
#include <math.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>

class Map : public rclcpp::Node {
  private:
    int width_;
    int height_;
    
    // A 1d array corresponding to the 2d grid. Each value is an occupancy probability [0=unoccupied, 100=occupied, -1=unknown]
    std::vector<int8_t> grid_;
    
    std::vector<double> origin_;     // A 2d coordinate in the odom frame that represents the point 0,0 in the 2d grid
    double resolution_;     // Meters per grid cell
    int occupied_val_;    // A threshold for what is considered occupied


  public:
    Map() : Node("mpc_node"), width_{-1}, height_{-1}, resolution_{-1} {
      //ros::param::get("/nav/occ_threshold", occupied_val_);
      occupied_val_ = this->get_parameter("/occ_threshold").as_int();
    }

    Map(const nav_msgs::msg::OccupancyGrid &map) : Node("mpc_node") { 
        update_map(map); 
        //ros::param::get("/nav/occ_threshold", occupied_val_);
        occupied_val_ = this->get_parameter("/occ_threshold").as_int();
    }

    // Find whether a 2d coordinate (in the real world) is occupied
    bool occupied_at_pos(std::vector<double> state) {
        assert(resolution_ != -1);
        assert(state.size() == 2);

        // adjust for map offset
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

        std::vector<int> grid_coords(state.begin(), state.end());
        
        // Convert to the 1d array index
        int flat_ind = height_ * grid_coords[1] + grid_coords[0];

        // if out of the map, it is occupied
        if (flat_ind < 0 || flat_ind >= grid_.size())
            return true;


        return grid_[flat_ind] > occupied_val_;
    }

    // Update the map when given a new occupancy grid
    void update_map(const nav_msgs::msg::OccupancyGrid &map) {
        resolution_ = map.info.resolution;
        width_ = map.info.width;
        height_ = map.info.height;
        origin_.push_back(map.info.origin.position.x);
        origin_.push_back(map.info.origin.position.y);
        grid_ = map.data;
    }
};

#endif