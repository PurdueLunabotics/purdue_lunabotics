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
From this point on, work in the docker container.

1. Go to the main directory
    * `cd /luna_ws`
1. Build the project
    * `colcon build --symlink-install`
    * You should see all packages built successfully. Some output to `stderr` is normal.
1. Set environment variables needed for ROS to function
    * `source /luna_ws/install/setup.bash`
    * You can add this to your `.bashrc` if not already present, so it runs every time you open a new terminal window.
1. Run the simulated robot
    * `ros2 launch lunabot_bringup simple_sim.launch`

    * This will start the simple simulation backend. The easiest way to view 
      the results is with [Foxglove](https://app.foxglove.dev). 
1. Open [Foxglove](https://app.foxglove.dev) in a browser, sign in, and click 'open connection.'

1. Select 'Foxglove Websocket' and make sure the URL is `ws://localhost:8765`. Click 'open.'

1. You should see the simulator running in real time. (You may have to adjust
foxglove's settings to change what's visible.)

![simple_sim_running_foxglove](todo)

## Important docs to read
- [Contributing Guidelines](contributing.md)