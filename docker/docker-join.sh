#!/bin/bash

XAUTHORITY_PATH="/home/$USER/.docker.tmp/.Xauthority"
if [ -f $XAUTHORITY_PATH ]; then 
    MAGIC_COOKIE=`xauth list $DISPLAY | awk '{print $3}'`
    X11PORT=`echo $DISPLAY | sed 's/^[^:]*:\([^\.]\+\).*/\1/'`
    xauth -f $XAUTHORITY_PATH add 172.17.0.1:$X11PORT . $MAGIC_COOKIE
    DISPLAY=`echo $DISPLAY | sed 's/^[^:]*\(.*\)/172.17.0.1\1/'`
fi
CONTAINER_NAME="${CONTAINER_NAME:-$USER-arc-rocket-league-dev}"

docker exec -it \
    -e DISPLAY \
    -e LIBGL_ALWAYS_INDIRECT \
    $@ \
    $CONTAINER_NAME \
    ${DOCKER_CMD:-/bin/zsh}
