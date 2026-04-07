# Version History

## v2.0.0 (2024.10.25)
- First version for Unitree Lidar L2

## v2.0.1 (2024.11.04)
- Update README.md
- Add library file for aarch environment
- Add support for ROS and ROS2

## v2.0.2 (2024.11.05)
- Add support for start and stop lidar rotation

## v2.0.3 (2024.11.14)
- Add udp connect information
- Modify `startLidar()` to `startLidarRotation()`, modify `stopLidar()` to `stopLidarRotation()`

## v2.0.4 (2024.12.10)
- Solve the problem of not able to start up lidar when it's in STANDBY mode

## v2.0.5 (2024.12.13)
- Add function `setLidarIpAddressConfig()` to set lidar ip address
- Add function `setLidarMacAddressConfig()` to set lidar mac address

## v2.0.6 (2024.12.19)
- Add function to get buffer size

## v2.0.7 (2024.12.19)
- Solve the bug to set lidar ip address correctly.

## v2.0.8 (2025.02.25)
- Add function `resetLidar()` to restart lidar hardware
- Modify namespace from `unitree_lidar_sdk` to `unilidar_sdk2`
- Modify library file name from `libunitree_lidar_sdk.a` to `libunilidar_sdk2.a`

## v2.0.9 (2025.03.04)
- Add function `sendUserCtrlCmd()` to send `LidarUserCtrlCmd` to lidar

## v2.0.10 (2025.03.10)
- Solve the bug of not displaying 2D LaserScan msg, 
- the default frame of 2D scan is changed to `unilidar_laserscan`