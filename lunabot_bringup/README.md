# lunabot_bringup

All the launch files related to running different parts of the software stack

### Gazebo simulator

#### Run it!

1. Double check you have the `realsense_ros_gazebo` and `mining_arena_gazebo` packages in [lunabot_sim](https://github.com/PurdueLunabotics/purdue_lunabotics/tree/simulator/lunabot_sim). If not, [run this](https://gist.github.com/raghavauppuluri13/e72f650116efa7a935161f4772083d10)

2. Run the [setup instructions](https://github.com/PurdueLunabotics/mining_arena_gazebo/tree/33f5949c90bd798f3925302800600661709a8f50#setup) for the [mining_arena_gazebo](https://github.com/PurdueLunabotics/mining_arena_gazebo) package to ensure gazebo can find all the gazebo model files
3. Run the simulator with `roslaunch lunabot_bringup sim.launch`

> Run with `debug:=true` (TODO: should run gdb, but it keeps exiting) and `verbose:=true` if you want gazebo's debug info

#### Usage

1. Get odometry information from the `odom` topic
2. Publish velocity control commands with `cmd_vel` topic (**NOTE: invert the `linear` and `angular` fields when you send it, its a weird bug that needs to be fixed TODO**)
3. Get 3D pointcloud and camera data from `d435_forward/depth/*` and `d435_backward/depth/*` topics

### Run rosbags

1. Download `localization_testing.bag` (link in Discord)
2. Make `lunabot_bringup/bags` folder and put the rosbag in there

#### View Localization

```
roslaunch lunabot_bringup test_localization.launch
```

#### View Map Building

```
roslaunch lunabot_bringup test_mapping.launch
```

### Run the robot

#### 1. Setup Networking

1. Connect router to power

> Optionally to get wifi, connect "internet" port on router to an external ethernet port with wifi

2. Connect computer to ethernet port on the router
3. Turn on the jetson and wait atleast 1.5 min
4. On the laptop, run `jetson` to run the ssh alias to the jetson. Then enter the password.

#### 2. Run roslaunch commands

1. In another terminal, set the ROS IP info on the laptop using the alias: `set_ip`
2. Launch the computer launch file on the laptop

```
roslaunch lunabot_bringup computer.launch
```

2. Launch the jetson launch file on the jetson

```
roslaunch lunabot_bringup dummybot.launch
```

> Can replace dummybot with `rollerbot` or for the config of the competition robot (compbot?)

That's it! Things should be running now.

#### More Info

- Can find all the exact alias commands in `~/.bash_aliases`
- During testing, we want the computer to be the ros master and the jetson to be a device that pushes messages to the computer, this is reflected in the `launch/ros_ip_setup.sh` shell file in this package
