// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef NAV2_MPPI_CONTROLLER__MODELS__TRAJECTORIES_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__TRAJECTORIES_HPP_

// xtensor creates warnings that needs to be ignored as we are building with -Werror
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>
#pragma GCC diagnostic pop

namespace mppi::models
{

/**
 * @struct mppi::models::State
 * @brief One state of the robot
 */
struct State
{
  float x, y, yaw, vx, wz;
};

constexpr unsigned int STATE_SIZE = sizeof(State) / sizeof(float);

/**
 * @class mppi::models::Trajectories
 * @brief A collection of Trajectories stored as an array of shape (batchSize, timesteps, STATE_SIZE)
 */
struct Trajectories
{
  xt::xtensor<float, 3> trajectories;

  /**
    * @brief Reset trajectories to zero
    */
  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    trajectories = xt::zeros<float>({batch_size, time_steps, STATE_SIZE});
  }
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__TRAJECTORIES_HPP_
