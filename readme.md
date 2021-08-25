# purdue_lunabotics

This is the git repo of the best Lunabotics software team 2020-2021

## Getting Started

### Prerequisites
- ROS must be installed
- ROS catkin workspace must be initialized

### Setup 
1. Clone this repo into src of ROS catkin workspace
```
git clone https://github.com/PurdueLunabotics/lunabotics_21.git
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
### Launch simulation robot
```
roslaunch lunabotics_robot robot.launch
```

### Launch on real robot
```
roslaunch lunabotics_robot robot.launch real:=true
```

#### Params
- real:=false (default) determines if you have hardware or just a simulation
- tessar:=true (default) determines if unzano camera (unzano) is connected
- unzano:=true (default) determines if tessar camera (tessar) is connected
- lidar:=true (default) determines if lidar is connected
