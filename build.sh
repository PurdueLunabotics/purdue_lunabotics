#!/bin/bash
cd ~/luna_ws && colcon build && cd src/purdue_lunabotics && source ~/luna_ws/install/setup.bash && ros2 launch $1 $2