### Docker Setup
This is intended for non-Mac, non-Windows, and non-Ubuntu 22.04 users. If you do not fall into this category, please follow the installation instructions on the main page.

#### 1. Install [Docker Engine]<https://docs.docker.com/engine/install
Follow the installation instructions for your operating system on this page. Make sure that you are installing Docker Engine, and not Docker Desktop, to avoid issues.

Following installation, open a terminal and run the following commands.
```
sudo systemctl start docker
sudo docker pull osrf/ros:humble-desktop-full
```

#### 2. Set up the workspace
Find a location you want to set up your project in. Run the following command.
```
mkdir -p luna/luna_ws/src
```
This creates the root directory of your ROS workspace (`luna_ws`) and the source code directory. Once completed, run the following commands to clone the repository.
```
cd ~/luna/luna_ws/src
git clone https://github.com/PurdueLunabotics/purdue_lunabotics.git
```

#### 3. Install the Dev Containers VSCode Extension
One of the easiest ways to work with Docker containers is by using the Dev Container feature of Visual Studio Code. If you have not downloaded it already, download VS Code. When in VSCode, find the "Extensions" menu, or press `Ctrl+Shift+X`. Find the "Dev Containers" extension and install it.

#### 4. Docker Container Setup
Open the `luna` folder in VSCode. Create a directory called `.devcontainer` and fill such that your file directory looks like the following.
```
luna/
|- .devcontainer/
|  |- Dockerfile
|  |- devcontainer.json
|- luna_ws/
|  |- src/
```

Paste the following into `Dockerfile`.
```
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    wget \
    gnupg \
    git \
    build-essential \
    cmake \
    libopencv-dev \
    libeigen3-dev \
    libssl-dev \
    libblas-dev \
    liblapack-dev \
    libgl1-mesa-dev \
    libglew-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    libegl1-mesa-dev \
    libpng-dev \
    libjpeg-dev \
    libboost-dev \
    libboost-serialization-dev \
    libboost-system-dev \
    libboost-filesystem-dev \
    python3-pip \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# ROS SETUP ---------------------------------------------------------------------------------------

RUN apt-get update
RUN apt-get install -y \
    ros-humble-desktop \
    ros-dev-tools \
    ros-humble-cv-bridge \
    python3-rosdep \
    python3-colcon-common-extensions && \
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    rosdep update


ENV ROS_SETUP="/opt/ros/humble/setup.bash"
ENV ROS_LOCAL_SETUP="/workspace/ros2_ws/install/setup.bash"

SHELL ["/bin/bash", "-c"]
RUN echo "source $ROS_SETUP" >> ~/.bashrc
RUN echo "source $ROS_LOCAL_SETUP" >> ~/.bashrc

# RTABMAP SLAM SETUP ------------------------------------------------------------------------------

RUN apt install -y ros-humble-rtabmap-ros
WORKDIR /workspace

RUN git clone https://github.com/introlab/rtabmap.git rtabmap
RUN git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git rtabmap_ros

RUN apt update && apt install -y ros-humble-grid-map ros-humble-grid-map-core ros-humble-grid-map-msgs
RUN rosdep update && rosdep install --from-paths . --ignore-src -r -y

# ROSDEP INSTALL ---------------------------------------------------------------------------------

COPY luna_ws/src/purdue_lunabotics /workspace/luna_ws/src/purdue_lunabotics
RUN source ${ROS_SETUP}

WORKDIR /workspace/ros2_ws
RUN rosdep install -i --from-paths src --rosdistro humble -y


# FINAL SETUP -------------------------------------------------------------------------------------

RUN export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
RUN . ~/.bashrc
```

After the Dockerfile has been copied, paste the following into `devcontainer.json`. Make sure to fill in your setup-specific information in the `"runArgs"` and `"postCreateCommand"` fields.
```
{
  "name": "ros-humble-luna",
  "dockerFile": "Dockerfile",
  "context": "..",
  "runArgs": [
      "--env", "DISPLAY=${localEnv:DISPLAY}", // Forward the host's DISPLAY
      "--volume", "/tmp/.X11-unix:/tmp/.X11-unix", // Mount the X11 socket
      "--volume", "${env:HOME}/.Xauthority:/root/.Xauthority", // Forward X11 authentication
      "-v", "<DIRECTORY WHERE YOUR LUNA FOLDER EXISTS>/luna:/workspace/",
      // "--gpus", "all", // only if you have a GPU - gives the container access to it
      "--privileged",
      "--env", "QT_QPA_PLATFORM=xcb" // Set the Qt platform plugin
  ],
  "customizations": {
    "vscode": {
      "extensions": [
        "ms-azuretools.vscode-docker",
        "ms-python.python",
        "redhat.vscode-xml",
        "ms-vscode.cmake-tools",
        "ms-vscode.cpptools",
        "ms-vscode.cpptools-extension-pack",
        "gruntfuggly.todo-tree",
        "charliermarsh.ruff"
      ]
    },
    "settings": {
      "C_Cpp.default.configurationProvider": "ms-vscode.cmake-tools"
    }
  },
  "remoteUser": "root", // Or your preferred user
  "workspaceFolder": "/workspace/",
  "postCreateCommand": "git config --global user.email \"<EMAIL ASSOCIATED WITH GITHUB>\" && git config --global user.name \"<NAME ON GITHUB ACCOUNT>\" && git config --global --add safe.directory /workspace/luna_ws/src/purdue_lunabotics",
}
```

#### 5. Build and Run
Once the files ae copied into the right places, make sure that you have opened the `luna` folder as the root directory in VSCode. Once here, open the Command Palette (`Ctrl+Shift+P`) and click on the _"Dev Containers: Rebuild and Reopen in Container"_ option. This should build for quite a while and then open into the running container.

Once this is complete, you can head back to the main page and follow the instructions starting with the _"Build + Source Manually"_ section.
