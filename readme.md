# purdue_lunabotics

This is the official ROS 2 git repo of the Purdue Lunabotics software team.

## Supported Platforms
- Linux
- Windows 10/11
- Mac OS (Some elements may not work on Apple Silicon)

## Requirements
You will need Docker to run the containerized workspace.
- MacOS: https://docs.docker.com/desktop/setup/install/mac-install/
- Windows: https://docs.docker.com/desktop/setup/install/windows-install/
  - Windows users may need to install [WSL](https://learn.microsoft.com/en-us/windows/wsl/install) to get Docker to work
- Linux: Follow your Distro's instructions

## Getting Started

#### 1. Clone the repository

`git clone https://github.com/PurdueLunabotics/purdue_lunabotics.git`

#### 2. Open the folder in [VS Code](https://code.visualstudio.com/)

#### 3. Install the `Dev Containers` extension if not already installed

#### 4. Create a folder in the workspace called `.devcontainer`

#### 5. Create a file called `.devcontainer/devcontainer.json` with this as its contents:
```
{
  "image": "ghcr.io/purduelunabotics/development:latest",
  "workspaceFolder": "/luna_ws/src/purdue_lunabotics",
  "workspaceMount": "source=${localWorkspaceFolder},target=/luna_ws/src/purdue_lunabotics,type=bind",
  "forwardPorts": [ 8765 ]
}
```

#### 5. Click 'Open a Remote Window' in the bottom left

#### 6. Click 'Reopen in Container'.

> Alternatively, you can run the docker container manually and work without VS Code.

## Building and Running the workspace

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
