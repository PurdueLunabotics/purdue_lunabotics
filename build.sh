#!/bin/bash
export GZ_VERSION=harmonic
cd ~/luna_ws && source ~/.bashrc && colcon build && cd src/purdue_lunabotics && source ~/luna_ws/install/setup.bash
