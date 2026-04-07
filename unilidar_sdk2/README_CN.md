# Unilidar SDK2

## 1. 简介

这个仓库是用于[Unitree L2](https://www.unitree.com/LiDAR) 激光雷达的软件开发包（SDK）。

您可以使用这个仓库中的代码接口来获取我们的激光雷达测量的点云IMU等数据，以及设置和获取激光雷达的相关配置参数。

我们为激光雷达提供了几种常用的接口：
- 原始的基于C++的SDK：[unitree_lidar_sdk](./unitree_lidar_sdk/README.md)
- 用于在ROS环境下解析和发布激光雷达数据的软件包：[unitree_lidar_ros](./unitree_lidar_ros/src/unitree_lidar_ros/README.md)
- 用于在ROS2环境下解析和发布激光雷达数据的软件包：[unitree_lidar_ros2](./unitree_lidar_ros2/src/unitree_lidar_ros2/README.md)


## 2. 坐标系定义

<div style="text-align:center">
  <img src="./docs/lidar_frame_definition.jpg" width="800">
</div>

本激光雷达的坐标系定义如上图所示，它满足右手坐标系的定义。设激光雷达点云坐标系为L，记IMU坐标系为I。

激光雷达点云坐标系的原点位于激光雷达底部安装面的中心，其+X轴与底部线缆出线方向相反，其+Y轴是通过将+X轴逆时针旋转90度得到的，其+Z轴垂直底面向上。

IMU坐标系的三个坐标轴方向与点云坐标系的对应坐标轴保持平行，二者仅存在原点位置的平移。IMU坐标系原点位于激光雷达点云坐标系的位置为（单位为米）：[-0.007698, -0.014655, 0.00667]。

按照标准的变换矩阵方式，从激光雷达点云坐标系L到IMU坐标系为I的位姿变换为：

$$
T_{LI} =
\begin{bmatrix}
1 & 0 & 0 & -0.007698 \\
0 & 1 & 0 & -0.014655 \\
0 & 0 & 1 & 0.00667 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

## 3. 如何使用C++ SDK

### 3.1 编译

您可以按照cmake项目的标准编译方式来编译此项目的示例程序：
```bash
cd unitree_lidar_sdk

mkdir build

cd build

cmake .. && make -j2
```

### 3.2 配置工作模式
激光雷达默认可以被配置为多种工作模式，包括标准FOV或广角FOV、3D模式或2D模式、IMU使能或者关闭、以太网或串口连接等。

我们可以可以通过上位机配置工作模式，或者也可使通过`unitree_lidar_sdk.h`中的如下接口来配置工作模式：
```
virtual void setLidarWorkMode(uint32_t mode) = 0;
```

设置工作模式通过一个`uint32_t`的整型变量实现，其每一个bit位对应一个功能的切换。按照Bit位置从低到高，对应位置为0或1对应的功能如下表所示：

|比特位|功能|取值0|取值1|
|---|---|---|----|
|0| 切换标准FOV和广角FOV | 标准FOV(180°) | 广角FOV(192°) |
|1| 切换3D和2D测量模式 | 3D测量模式 | 2D测量模式 |
|2| 使能或关闭IMU| 使能IMU | 关闭IMU |
|3| 切换网口模式和串口模式 | 网口模式 | 串口模式 |
|4| 切换激光雷达上电默认启动模式 | 上电即自行启动 | 上电保持不转动并等待启动命令 |
|5-31| 保留 | 保留 | 保留 |

常用的使用模式为标准FOV+3D测量+使能IMU+上电自启动，即这几个bit位置均保持为0即可，我们只需要确定是使用网口连接模式还是串口连接模式。例如，在下面的示例程序中，
- 如果使用网口连接模式，则将工作模式配置为0（即整型变量的所有bit位置均等于0）
- 如果使用串口连接模式，则将工作模式配置为8（即整型变量的第3个bit位置等于1，其余0-31位的bit位置均等于0）

默认出厂的激光雷达工作模式为0，即网口以太网的通信模式。

### 3.3 使用网口运行

使用以太网的连接方式运行激光雷达的示例程序为：`example_lidar_udp.cpp`。

首先，使用网线将您的激光雷达连接到计算机，然后确认将您的电脑的对应网卡配置为激光雷达的默认目标IP地址：
```bash
192.168.1.2
```

然后，运行示例程序：
```bash
../bin/example_lidar_udp
```

示例输出如下：
```
$ ../bin/example_lidar_udp 
Unilidar initialization succeed!
set Lidar work mode to: 0
lidar hardware version = 1.1.1.1
lidar firmware version = 2.3.3.0
lidar sdk version = 2.0.2
stop lidar rotation ...
start lidar rotation ...
dirty percentage = 1.616020 %
time delay (second) = 0.001880
An IMU msg is parsed!
    system stamp = 1730191291.3044135571
    seq = 87, stamp = 1730191291.304411172
    quaternion (x, y, z, w) = [-0.3645, 0.0077, 0.0099, 0.9293]
    angular_velocity (x, y, z) = [0.0209, -0.0644, 0.0146]
    linear_acceleration (x, y, z) = [0.2215, 0.3537, 9.5822]
An IMU msg is parsed!
    system stamp = 1730191291.3070139885
    seq = 88, stamp = 1730191291.307010650
    quaternion (x, y, z, w) = [-0.3646, 0.0077, 0.0099, 0.9293]
    angular_velocity (x, y, z) = [0.0086, -0.0039, 0.0122]
    linear_acceleration (x, y, z) = [0.1608, 0.3222, 9.7098]
A Cloud msg is parsed! 
	stamp = 1730724860.502892, id = 32
	cloud size  = 5205, ringNum = 18
	first 10 points (x,y,z,intensity,time,ring) = 
	  (1.554845, -1.070981, 0.000000, 0.000000, 0.000000, 1)
	  (1.645649, -1.132876, 0.020923, 0.000000, 0.000008, 1)
	  (1.698396, -1.168516, 0.043183, 0.000000, 0.000015, 1)
	  (1.981559, -1.362552, 0.075574, 0.000000, 0.000023, 1)
	  (1.981164, -1.361497, 0.100753, 0.000000, 0.000031, 1)
	  (1.942685, -1.334285, 0.123513, 0.000000, 0.000039, 1)
	  (1.602043, -1.099691, 0.122253, 0.000000, 0.000046, 1)
	  (1.612709, -1.106375, 0.143620, 0.000000, 0.000054, 1)
	  (1.616608, -1.108412, 0.164594, 0.000000, 0.000062, 1)
	  (1.619501, -1.109757, 0.185582, 0.000000, 0.000069, 1)
	  ...
```

如果您需要修改激光雷达的默认IP地址，可以参考使用说明书使用我们的上位机修改。

### 3.4 使用串口运行

注意，默认出厂的激光雷达为网口通信模式。如果您尚未将通信模式切换为串口通信模式，则您需要先使用网口连接才能和激光雷达通信上，从而才可以将其通信方式切换为串口通信模式。您可以使用我们的上位机进行切换，或者也可以参考[4.2]部分的内容，先使用默认的网口通信示例程序`example_lidar_udp.cpp`和激光雷达进行通信，并将激光雷达工作模式设置为串口通信模式（`workMode=8`），再断电重启激光雷达即可。

在确认当前激光雷达为串口通信模式后，我们使用串口线将激光雷达连接到计算机，然后确认激光雷达的串口名称，默认为如下值：
```
"/dev/ttyACM0"
```

您也可以自行修改默认的串口名称，并在示例代码`example_lidar_serial.cpp`中对应修改。

编译后，可以运行示例程序：
```bash
../bin/example_lidar_serial
```

示例输出如下：
```
$ ../bin/example_lidar_serial
Unilidar initialization succeed!
set Lidar work mode to: 0
lidar hardware version = 1.1.1.1
lidar firmware version = 2.3.3.0
lidar sdk version = 2.0.2
stop lidar rotation ...
start lidar rotation ...
dirty percentage = 1.363776 %
time delay (second) = 0.002044
An IMU msg is parsed!
    system stamp = 1730191291.3044135571
    seq = 87, stamp = 1730191291.304411172
    quaternion (x, y, z, w) = [-0.3645, 0.0077, 0.0099, 0.9293]
    angular_velocity (x, y, z) = [0.0209, -0.0644, 0.0146]
    linear_acceleration (x, y, z) = [0.2215, 0.3537, 9.5822]
An IMU msg is parsed!
    system stamp = 1730191291.3070139885
    seq = 88, stamp = 1730191291.307010650
    quaternion (x, y, z, w) = [-0.3646, 0.0077, 0.0099, 0.9293]
    angular_velocity (x, y, z) = [0.0086, -0.0039, 0.0122]
    linear_acceleration (x, y, z) = [0.1608, 0.3222, 9.7098]
A Cloud msg is parsed! 
	stamp = 1730724860.419490, id = 31
	cloud size  = 5217, ringNum = 18
	first 10 points (x,y,z,intensity,time,ring) = 
	  (-0.447932, 0.172502, 0.000000, 0.000000, 0.000000, 1)
	  (-0.442354, 0.170217, 0.004964, 0.000000, 0.000008, 1)
	  (-0.443261, 0.170429, 0.009948, 0.000000, 0.000015, 1)
	  (-0.445051, 0.170981, 0.014983, 0.000000, 0.000023, 1)
	  (-0.443060, 0.170079, 0.019891, 0.000000, 0.000031, 1)
	  (-0.442887, 0.169877, 0.024860, 0.000000, 0.000039, 1)
	  (-0.442665, 0.169655, 0.029825, 0.000000, 0.000046, 1)
	  (-0.447052, 0.171199, 0.035154, 0.000000, 0.000054, 1)
	  (-0.442076, 0.169157, 0.039747, 0.000000, 0.000062, 1)
	  (-0.446358, 0.170658, 0.045172, 0.000000, 0.000069, 1)
	  ...
```

## 4. 如何使用ROS软件包

### 4.1 依赖
依赖项包括`PCL`和`ROS`。

我们已经验证这个包可以在以下环境中成功运行：
- `Ubuntu 20.04`
- `ROS noetic`
- `PCL-1.10`
- `unitree_lidar_sdk`

建议您配置类似这样的环境来运行这个包。

### 4.2 配置

默认的激光雷达通信方式为网口模式，如果您需要修改工作模式，您需要修改配置文件中的对应参数。配置文件路径为：
```
unitree_lidar_ros/src/unitree_lidar_ros/config/config.yaml
```

如果您有特殊需求，例如更改云话题名称或IMU话题名称，您也可以在配置文件中进行配置。

默认的云话题及其坐标系名称是：
- 话题名称："unilidar/cloud"
- 坐标系："unilidar_lidar"

默认的IMU话题及其坐标系名称是：
- 话题名称："unilidar/imu"
- 坐标系："unilidar_imu"

### 4.3 编译

编译：

```
cd unitree_lidar_ros

catkin_make
```

### 4.4 运行

运行：
```
source devel/setup.bash

roslaunch unitree_lidar_ros run.launch
```

在Rviz窗口中，您将看到我们的激光雷达点云如下：

![img](./docs/ros1_cloud.png)

## 5. 如何使用ROS2软件包

### 5.1 依赖

依赖项包括`PCL`和`ROS2`。

我们已经验证这个包可以在以下环境中成功运行：
- `Ubuntu 20.04` 
- `ROS2 foxy`
- `PCL-1.10`
- `unitree_lidar_sdk`

建议您配置类似这样的环境来运行这个包。

### 5.2 配置

默认的激光雷达通信方式为网口模式，如果您需要修改工作模式，您需要修改配置文件中的对应参数。配置文件路径为：
```
unitree_lidar_ros2/src/unitree_lidar_ros2/launch/launch.py
```

如果您有特殊需求，例如更改云话题名称或IMU话题名称，您也可以在配置文件中进行配置。

默认的云话题及其坐标系名称是：
- 话题名称："unilidar/cloud"
- 坐标系："unilidar_lidar"

默认的IMU话题及其坐标系名称是：
- 话题名称："unilidar/imu"
- 坐标系："unilidar_imu"

### 5.3 编译

编译：

```
cd unilidar_sdk/unitree_lidar_ros2

colcon build
```

### 5.4 运行

运行：
```
source install/setup.bash

ros2 launch unitree_lidar_ros2 launch.py
```

在Rviz窗口中，你将看到我们的激光雷达点云如下：

![img](./docs/ros2_cloud.png)

## 6. 如何解析原始数据包

如果您希望自己解析原始的网口数据或者串口数据以得到点云和IMU等数据，您可以参考我们自定义的通信协议进行解析。

具体地，您可以参考我们的使用说明书，以及`unitree_lidar_sdk`下的如下文件：
```
unitree_lidar_protocol.h
unitree_lidar_utilities.h
```
