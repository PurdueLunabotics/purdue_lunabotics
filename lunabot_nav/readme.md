# lunabot_nav

## Quick Start

>> This tutorial assumes you are running linux

1. Setup repo in main readme

2. Ensure you have built and sourced your catkin workspace

3. Connect the lidar to your computer and check if its connected by checking the connected devices

```
ls /dev | grep ttyUSB
```

4. Add the write authority of the lidar device you just found: (such as /dev/ttyUSB0)

sudo chmod 666 /dev/ttyUSB0

5. Run the RPLiDAR launch file 

```
roslaunch lunabot_nav lidar.launch
```

You should see a Rviz window pop up with the lidar pointcloud scans

### See the raw data

1. In another terminal shell, list all the topics

```
rostopic list
```

> Take note of the message type and individual data types, then google what you find.

2. Find the topics related to the lidar sensor and see the live raw output

```
rostopic echo /scan 
```

## More information

Check out the [rplidar_ros](http://wiki.ros.org/rplidar) package
