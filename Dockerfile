FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/New_York
ENV GZ_VERSION=harmonic

EXPOSE 8765

SHELL ["/bin/bash", "-c"]

RUN apt update -y && \
    apt install software-properties-common -y && \
    add-apt-repository universe

RUN apt update -y && \
    apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update -y && \
    apt upgrade -y

RUN apt install -y ros-jazzy-desktop ros-dev-tools python3-colcon-common-extensions

RUN apt install -y ros-jazzy-rmw-zenoh-cpp

RUN apt install -y clang

RUN cat <<EOF > ~/.bashrc
source /opt/ros/jazzy/setup.bash
source /luna_ws/install/setup.bash

export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:\$(ros2 pkg prefix lunabot_sim)/share
export ZENOH_ROUTER_CHECK_ATTEMPTS=0
export ZENOH_SESSION_CONFIG_URI="/luna_ws/src/purdue_lunabotics/session_config.json5"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
EOF

RUN apt update -y && apt install -y \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-rtabmap-ros \
  libusb-dev \
  curl \
  lsb-release gnupg

RUN apt-get update && \
    apt-get install -y ros-jazzy-ros-gz

RUN source /opt/ros/jazzy/setup.bash && \
    rosdep init && \
    apt update -y && \
    rosdep update

COPY ./ /luna_ws/src/purdue_lunabotics

RUN source /opt/ros/jazzy/setup.bash && \
    apt update -y && \
    rosdep install -r --from-paths /luna_ws/src --ignore-src --rosdistro jazzy -y

VOLUME /luna_ws/src/purdue_lunabotics
