# purdue_lunabotics

This is the official git repo of the Purdue Lunabotics software team.

## Supported Platforms
- Ubuntu 20.04

## Quick Start

1. [Install ROS](https://wiki.purduearc.com/wiki/tutorials/setup-ros)

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


## Important docs to read
- [contributing guidelines](https://github.com/PurdueLunabotics/purdue_lunabotics/blob/master/contributing.md)
- [running_the_robot](https://github.com/PurdueLunabotics/purdue_lunabotics/blob/master/running_the_robot.md)
- [firmware setup](https://github.com/PurdueLunabotics/purdue_lunabotics/blob/master/lunabot_embedded/readme.md)
