// not used right now, for costmap layer

namespace nav2_crater_costmap_plugin
{

CraterCostmap::CraterCostmap()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{

}

void CraterCostmap::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{

    unsigned int size_x = Costmap2D.getSizeInCellsX(), size_y = Costmap2D.getSizeInCellsY();

    min_i = std::max(0, min_i);
    min_j = std::max(0, min_j);
    max_i = std::min(static_cast<int>(size_x), max_i);
    max_j = std::min(static_cast<int>(size_y), max_j);

    int craterLayer;

    updateWithAddition()
} 

}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_gradient_costmap_plugin::CraterCostmap, nav2_costmap_2d::Layer)