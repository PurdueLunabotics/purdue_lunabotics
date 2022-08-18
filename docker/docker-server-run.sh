#!/bin/bash

WS_DIR=$(realpath $(dirname $0)/../../../)
REPO_NAME="purduelunabotics/2022-2023"
CONTAINER_NAME="${CONTAINER_NAME:-lunabotics-dev}"
echo "mounting host directory $WS_DIR as container directory /home/$USER/catkin_ws"

# tty-specific options
if [ -t 0 -a -t 1 ]
then
    TTY_OPTS="-it"
fi

if [ "$(docker ps -a | grep $USER-$CONTAINER_NAME)" ]
then
    printf "\x1b[1;31mcontainer already running\nyou can rejoin it with:\x1b[1;36m docker-join.sh\x1b[1;31m\nor kill the current container with:\x1b[1;36m docker kill $USER-$CONTAINER_NAME\n"
    exit
fi

$WS_DIR/src/rocket_league/docker/docker-build.sh

#XAUTH VOODOO
rm -rf /home/$USER/.docker.tmp
mkdir /home/$USER/.docker.tmp
cp /home/$USER/.Xauthority /home/$USER/.docker.tmp/
X11PORT=`echo $DISPLAY | sed 's/^[^:]*:\([^\.]\+\).*/\1/'`
MAGIC_COOKIE=`xauth list $DISPLAY | awk '{print $3}'`
xauth -f /home/$USER/.docker.tmp/.Xauthority add 172.17.0.1:$X11PORT . $MAGIC_COOKIE
DISPLAY=`echo $DISPLAY | sed 's/^[^:]*\(.*\)/172.17.0.1\1/'`

docker run --rm \
    $TTY_OPTS \
    -e USER \
    -e DISPLAY \
    -e XAUTHORITY=`echo /home/$USER/.docker.tmp/.Xauthority` \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /home/$USER/.docker.tmp/:$HOME/.docker.tmp:ro \
    -v $WS_DIR:/home/$USER/catkin_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    `[ -f ~/.gitconfig ] && echo "-v $HOME/.gitconfig:/home/$USER/.gitconfig:ro"` \
    -v ~/.ssh:/home/$USER/.ssh:ro \
    --name $USER-$CONTAINER_NAME \
    $@ \
    $REPO_NAME:local-$USER \
    ${DOCKER_CMD:+/bin/zsh -c "$DOCKER_CMD"}

#Remove image to save space
docker rmi $REPO_NAME:local-$USER
