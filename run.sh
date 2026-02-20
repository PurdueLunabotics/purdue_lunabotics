#!/bin/bash
export GZ_VERSION=harmonic
cd ~/luna_ws && colcon build && source ~/.bashrc && cd src/purdue_lunabotics && source ~/luna_ws/install/setup.bash && ros2 launch $@
