// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "custom_mppi_controller/critics/custom_critic.hpp"
#include <cmath>

constexpr float PI = 3.141592;

namespace mppi::critics
{

using xt::evaluation_strategy::immediate;

void CustomCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(weight_, "cost_weight", 5.0f);

  RCLCPP_INFO(logger_, "CustomCritic instantiated with %f weight.", weight_);
}

void CustomCritic::score(CriticData & data)
{
  if (!enabled_)
  {
    return;
  }

  const geometry_msgs::msg::Pose goal = data.goal;

  const double& goal_x = goal.position.x;
  const double& goal_y = goal.position.y;

  const auto traj_x = xt::view(data.trajectories.trajectories, xt::all(), xt::all(), 0);
  const auto traj_y = xt::view(data.trajectories.trajectories, xt::all(), xt::all(), 1);

  data.costs += (xt::sum(xt::hypot(traj_x - goal_x, traj_y - goal_y),{1}, immediate) * weight_);


  for (unsigned int i = 0; i < data.trajectories.trajectories.shape(0); i++) {
    for (unsigned int j = 0; j < data.trajectories.trajectories.shape(1); j++) {

      float trajX = data.trajectories.trajectories(i, j, 0);
      float trajY = data.trajectories.trajectories(i, j, 1);
      float yaw = data.trajectories.trajectories(i, j, 2);

      float targetAngle = std::atan2(goal_y - trajY, goal_x - trajX);

      float angularDiff = 1.0 - std::cos(targetAngle - yaw);
      float distToGoal = (std::sqrt(std::pow(trajX-goal_x, 2) + std::pow(trajY-goal_y, 2)));
      if (angularDiff < 0.1 || distToGoal < 0.1) {
        angularDiff = 0;
      }

      data.costs(i) += angularDiff * 10.0;

    }
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::CustomCritic, mppi::critics::CriticFunction)
