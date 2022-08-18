# Docker Development Image Setup Guide

## You have to read this
- Run `docker-build.sh` to build the development image.
- Run `docker-run.sh` to start the container.
- Run `docker-server-run.sh` to start the container when running on a remote computer connected via `ssh -Y`.
- Run `docker-join.sh` to join the currently running container.
- If you edit `Dockerfile.local`, re-run `docker-build.sh`.
- To add a ROS dependency, edit `Dockerfile`
- If you edit `Dockerfile`, run `docker-build-base.sh`, then `docker-build.sh`.

## You don't have to read this

### Remote Image
`Dockerfile` is used to build `purduearc/rocket-league:latest`.
[DockerHub](https://hub.docker.com/repository/docker/purduearc/rocket-league/)
is set up to automatically build this image when changes are made to the `main`
branch to this repository. You can build this image locally by running
`docker-build-base.sh`. You might do this if you make any changes to
`Dockerfile`, such as adding a system library from `apt` or if you need to
install anything from source. This image uses a generic user and does not
include any code from this repository or install any ROS dependencies other than
those found in the standard `ros-melodic-desktop-full` install.

### Local Image
`Dockerfile.local` is used to build `purduearc/rocket-league:local`. DockerHub
will not automatically build this image, you have to do that yourself by running
`docker-build.sh`. Running this will add your user so files can be accessed and
modified from both the container and your bare-metal computer.

### Running the image
Run `docker-run.sh` to start the image (`purduearc/rocket-league:local`) and
mount your local catkin workspace. You can pass in additional arguments directly
to the `docker run` command. For example, if you have an Nvidia GPU and have
[`docker-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
installed, you can use your GPU from within the container by using `--gpus all`.
Other useful commands are `--net host` and `--priviledged` for access to host
networking and peripheral devices respectively. You can also pass in a command
through the `$DOCKER_CMD` environment variable, rather than loading into zsh.
For example:
  - `docker-run.sh`: Run the container and start the shell
  - `docker-run.sh --gpus all`: Run the container with Nvidia GPU support
  - `docker-run.sh --gpus all --net host --priviledged`: Run the container with
  Nvidia GPU support, host networking, and complete access to peripherals
  - `DOCKER_CMD=roscore docker-run.sh`: Run the container and start the ROS core
  - `DOCKER_CMD='roslaunch my_package my_launch_file.launch' docker-run.sh
  --gpus all --net host --priviledged`: Run a specific launch
  file with Nvidia GPU support, host networking, and complete access to
  peripherals
  
  ### Server Xauth voodoo explanation
  - Remove the old temporary xauth folder
  - Create the temporary xauth folder
  - Copy the current Xauthority file to temp directory to not pollute the main Xauthority file
  - Get the port that the display is running on
  - Get the cookie for the display that needs to be copied
  - Add the copied cookie to the temporary Xauthority file using the docker interface ip as the display location
  - Set the display variable for inside the container to use the new display location

### Other things
You can also run `docker-join.sh` to attach a shell to the container. You can
pass in arbitrary arguments similar to `docker-run.sh`.

If you have to use an external camera (not including webcams) you need to
configure permissions. Run `udev-aravis.sh` on your bare-metal computer (not in
the container) to do this. You only have to do this once.
