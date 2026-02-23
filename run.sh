#!/bin/bash
export GZ_VERSION=harmonic
<<<<<<< Updated upstream
cd ~/luna_ws && source ~/.bashrc &&colcon build --symlink-install && cd src/purdue_lunabotics && source ~/luna_ws/install/setup.bash && ros2 launch $@
=======
cd ~/luna_ws && source ~/.bashrc && colcon build --symlink-install && cd src/purdue_lunabotics && source ~/luna_ws/install/setup.bash && ros2 launch $@
>>>>>>> Stashed changes
