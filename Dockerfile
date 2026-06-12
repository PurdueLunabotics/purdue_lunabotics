FROM ros:jazzy

EXPOSE 8765

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y ros-jazzy-rmw-zenoh-cpp clang libusb-dev curl lsb-release gnupg

RUN cat <<EOF > ~/.bashrc
source /opt/ros/jazzy/setup.bash
source /luna_ws/install/setup.bash

export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:\$(ros2 pkg prefix lunabot_sim)/share
export ZENOH_ROUTER_CHECK_ATTEMPTS=0
export ZENOH_SESSION_CONFIG_URI="/luna_ws/src/purdue_lunabotics/session_config.json5"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
EOF

RUN source /opt/ros/jazzy/setup.bash && \
    apt update && \
    rosdep update

COPY ./ /luna_ws/src/purdue_lunabotics

RUN source /opt/ros/jazzy/setup.bash && \
    apt update -y && \
    rosdep install -r --from-paths /luna_ws/src --ignore-src --rosdistro jazzy -y --skip-keys="ros_gz_bridge ros_gz_sim"

VOLUME /luna_ws/src/purdue_lunabotics
