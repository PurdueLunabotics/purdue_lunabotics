# purdue_lunabotics

This is the official git repo of the Purdue Lunabotics software team.

## Supported Platforms
- Ubuntu 20.04
- if on windows:
  - Use WSL2 (caveat: gui will appear more laggy) 
  - dual boot linux (caveat: can be hard to set up, not recommended for beginners)
- mac or windows (not tested):
  - [try robostack to install ROS](https://robostack.github.io/GettingStarted.html)
     - Once you complete this, move to step 2

## Getting started

1. Install ROS (Linux only)

[ROS install:](http://wiki.ros.org/ROS/Installation/TwoLineInstall/)
```
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
```

Install catkin tools + setup catkin workspace:
```
mkdir ~/catkin_ws/src
sudo apt install python3-catkin-tools
```

This tutorial assumes you have your `catkin_ws` initialized in your home directory: `~/catkin_ws`

2. Clone the repository to `src` folder of ROS workspace

```
cd ~/catkin_ws/src
git clone https://github.com/PurdueLunabotics/purdue_lunabotics.git
```

3. Install dependencies at the root of your catkin workspace
```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
sudo ./src/purdue_lunabotics/install-non-ros-deps.sh
```

4. Build + source (Do this every time you download new packages)

```
catkin build
source ~/catkin_ws/devel/setup.bash # or .zsh if you use a zsh terminal
```
> Note: Build + source every time you add new packages. Source every time you open a fresh terminal, or add the line to your ~/.bashrc (or .zshrc) so it sources automatically

## Testing your setup

1. Run the sim

```
roslaunch lunabot_bringup sim.launch
```

> you should see two new windows pop up: once called gazebo and one called rviz

2. Set goal waypoint and watch the robot navigate


## Important docs to read
- [contributing guidelines](https://github.com/PurdueLunabotics/purdue_lunabotics/blob/master/contributing.md)
- [running_the_robot](https://github.com/PurdueLunabotics/purdue_lunabotics/blob/master/running_the_robot.md)
- [firmware setup](https://github.com/PurdueLunabotics/purdue_lunabotics/blob/master/lunabot_embedded/readme.md)
