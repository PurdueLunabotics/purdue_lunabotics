# lunabot_localization

## Quick Start

1. Setup repo in main readme

2. Ensure you have built and sourced your catkin worskpace

3. Connect the T265 localization camera to your computer and check if its connected

```
# Linux
ls /dev | grep ttyUSB
# or 
ls /dev | grep ttyACM 
# or
lsusb

# Mac
ioreg -p IOUSB -l -w 0
```
After running these, should see some output and can try to discern if the camera gets recognized. If not, try a different computer (ideally that runs linux).

4. Run the localization launch file to run Apriltag detections and T265 output data 

```
roslaunch lunabot_localization localization.launch
```

You should see an Rviz window pop up 

### Visualize outputs of detections

In another window (make sure to source ROS workspace in the new env if not done automatically):
```
rqt
```

6. Click on `Plugins > Visualization > Image View` 
7. Select the `/tag_detections_image` topic and see the camera output of the detections. If you have the board of apriltags, put that in the frame view and see the detections 

### See the raw data

1. In another terminal shell, list all the topics

```
rostopic list
```

> Take note of the message type and individual data types, then google what you find.

2. Find the topics related to  sensor and see the live raw output

```
rostopic echo /tag_detections 
```

## More information

Check out the [apriltag_ros](http://wiki.ros.org/apriltag_ros) package
Check out the [realsense_ros](https://github.com/IntelRealSense/realsense-ros#using-t265) package
