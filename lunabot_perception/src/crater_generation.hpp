// not used right now, for costmap layer

#ifndef CRATER_COSTMAP_HPP_
#define CRATER_COSTMAP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace nav2_gradient_costmap_plugin : public nav2_costmap_2d::Layer
{

class CraterCostmap{
    private:
    rclcpp::Logger logger;

    public:
    CraterCostmap();
    onInitialize();
    
    void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y);

    void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

    void reset()
    {
        return;
    }

    void onFootprintChanged();

    bool isClearable() {
        return false;
    }

    private:
    double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

    // Indicates that the entire gradient should be recalculated next time.
    bool need_recalculation_;
};
}

#endif