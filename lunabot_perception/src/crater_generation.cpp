// I'm going to make so many memory leaks
#include nav2_costmap_2d

namespace nav2_crater_costmap_plugin
{

CraterCostmap::CraterCostmap()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

void CraterCostmap::onInitialize(){
    need_recalculation_ = false;
    current_ = true;
}

void CraterCostmap::onFootprintChanged(){
    need_recalculation_ = true;
}

void CraterCostmap::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
    CostmapLayer.updateWithAddition(master_grid, min_i, min_j, max_i, max_j)
} 

}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_gradient_costmap_plugin::CraterCostmap, nav2_costmap_2d::Layer)