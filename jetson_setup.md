# New Jetson Setup

1. Make sure you have an NVME SSD

2. Flash the SSD with Ubuntu OS using the [JetsonHacks tutorial](https://jetsonhacks.com/2023/05/30/jetson-orin-nano-tutorial-ssd-install-boot-and-jetpack-setup/)

> Make sure you have 40-80GB of free space on the host computer

3. Install ROS [Offical Noetic install](http://wiki.ros.org/noetic/Installation/Ubuntu)

4. Install catkin 

```
sudo apt install python3-catkin-tools
```

5. Create a catkin workspace

```
mkdir /catkin_ws
cd /catkin_ws
catkin init
```

6. Clone the `purdue_lunabotics` repo. Follow setup instructions in readme
