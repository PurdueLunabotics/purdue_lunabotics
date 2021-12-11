# lunabot_bringup

### Run rosbag

1. Download rosbag (in Discord)
2. Launch rviz

```
roslaunch lunabot_bringup view_tf.launch
```
3. Play the rosbag
```
rosbag play <path to rosbag>
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
