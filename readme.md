### purdue_lunabotics

This is the git repo of the best Lunabotics software team.

## Getting Started
# Prerequisites
- ROS must be installed
- ROS catkin workspace must be initialized

# Setup 
1. Clone this repo into src of ROS catkin workspace
```
git clone https://github.com/PurdueLunabotics/Software_2020-2021_ros.git
```
2. Build workspace
```
catkin build
```
or
```
cd ~/catkin_ws
catkin_make
```
3. Source workspace
```
source ~/catkin_ws/devel/setup.bash
```
# Launch robot with all sensors in Rviz

```
roslaunch purdue_lunabotics_ros robot.launch
```

