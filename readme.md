# purdue_lunabotics

This is the official git repo of the Purdue Lunabotics software team.

## Supported Platforms Out of the Box
- Fully-supported
  - Ubuntu 20.04
- Partly-supported
  - Windows:
    - Use WSL2 (caveat: gui will appear more laggy) 
    - Dual boot linux (caveat: can be hard to set up, not recommended for beginners)
- Not supported
  - MacOS (both Intel and M1)

## Other installation methods 
- Docker (native support coming soon), adds support for MacOS and Windows
    - [Get docker](https://docs.docker.com/get-docker/)
    - [Use this ROS noetic Docker image (choose arm if you're on M1 Mac)](https://hub.docker.com/layers/library/ros/noetic/images/sha256-41a0aad743d47e08bec68cf48005706c27a3d7aad10632d204cada99ef3642b2?context=explore)
    - [Useful docker-ros tutorial for learning how docker can be used with ROS](https://roboticseabass.com/2021/04/21/docker-and-ros/) 

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
git clone --recurse-submodules https://github.com/PurdueLunabotics/purdue_lunabotics.git 
```

> Note: if you forgot to add recurse, run: `git submodule update --init --recursive --remote`

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
> NOTE: to remove the TF_REPEATED lines run:`{ roslaunch lunabot_bringup sim.launch verbose:=true 2>&1 | grep -Ev 'TF_REPEATED_DATA|buffer_core|at line|^$'; } 2>&1`

2. Set goal waypoint in rviz and watch the robot navigate


## Important docs to read
- [contributing guidelines](https://github.com/PurdueLunabotics/purdue_lunabotics/blob/master/contributing.md)
- [running_the_robot](https://github.com/PurdueLunabotics/purdue_lunabotics/blob/master/running_the_robot.md)
- [firmware setup](https://github.com/PurdueLunabotics/purdue_lunabotics/blob/master/lunabot_embedded/readme.md)
- [simulated arena docs](https://github.com/PurdueLunabotics/mining_arena_gazebo/blob/master/README.md)
