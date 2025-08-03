# purdue_lunabotics

This is the official ROS 2 git repo of the Purdue Lunabotics software team.

## Supported Platforms
- Ubuntu 22.04
- Windows and Mac:
  - Ubuntu 22.04 is the only *fully* supported platform. There are a few ways to run a virtual machine with linux on it. 
    - Windows: [WSL2](https://ubuntu.com/desktop/wsl) (caveat: strange interactions with GPUs may make GUIs stutter) 
    - Dual boot linux (caveat: can be hard to set up, not recommended for beginners)
    - [Docker](https://docs.docker.com/get-docker/) containers (caveat: not tested)

## Getting started

### One Line Install:
Run the following command to install everything (Must be in Ubuntu 22.04)
```
wget -c https://raw.githubusercontent.com/PurdueLunabotics/purdue_lunabotics/refs/heads/ros2-develop/install.sh && chmod +x ./install.sh && ./install.sh
```

### 1. Install Ubuntu 22.04

### 2. Install ROS 2:

Follow this
[Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) directly. For convenience, the steps are also here:

Ensure Ubuntu Universe repository is enabled.
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Now add the ROS 2 GPG key with apt.
```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then add the repository to your sources list.
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Update apt.
```
sudo apt update
sudo apt upgrade
```

Install Ros and Dev tools.
```
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

Setup your ros environment:
```
source /opt/ros/humble/setup.bash
```
It is also recommended to put this line in your `.bashrc` file, so that it runs every time you start a terminal.

### 3. Set up workspace
Install `colcon`, the build system
```
sudo apt install python3-colcon-common-extensions
```

Make a workspace directory with a `src` directory in it. This can be named anything. For example,
```
mkdir -p luna_ws/src
```

Clone the repository in the `src` folder.
```
cd ~/luna_ws/src
git clone https://github.com/PurdueLunabotics/purdue_lunabotics.git
```

Install dependencies at the root of your workspace
```
cd ~/luna_ws
rosdep install -i --from-path src --rosdistro humble -y
```
<!-- TODO: Non-ros deps -->
Install other dependencies
```
sudo apt install ros-humble-gz-ros2-control ros-humble-joint-state-publisher ament-cmake-python ament-cmake curl lsb-release gnupg
```
Install Gazebo Harmonic
```
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic ros-humble-ros-gzharmonic
```

Build + Source Manually
```
cd ~/luna_ws
colcon build
source ~/luna_ws/install/setup.bash # or .zsh if you use a zsh terminal
```
OR use script
```
cd ~/luna_ws/src/purdue_lunabotics
source build.sh
```
> Note: Build + source every time you change anything or open a new terminal. If building manually, make sure to do it within the correct folder

## Testing your setup

1. Run the sim

```
ros2 launch lunabot_bringup sim.launch
```
> you should see two new windows pop up: once called gazebo and one called rviz

2. Set goal waypoint in rviz and watch the robot navigate
![mpc_fix_gazebo_skid_steer](https://github.com/PurdueLunabotics/purdue_lunabotics/assets/41026849/a5cdaf41-f482-4b47-bd7b-bc8b7cb88880)

<!-- TODO: fix any links once these things exist -->
## Important docs to read
- [contributing guidelines](contributing.md)
- [running_the_robot](https://github.com/PurdueLunabotics/purdue_lunabotics/blob/master/running_the_robot.md) 
- [firmware setup](https://github.com/PurdueLunabotics/purdue_lunabotics/blob/master/lunabot_embedded/readme.md)
- [simulated arena docs](https://github.com/PurdueLunabotics/lunabot_sim/blob/master/README.md)
