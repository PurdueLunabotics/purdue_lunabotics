#!/bin/bash

DOCKER_DIR=$(realpath $(dirname $0))
REPO_NAME="purduelunabotics/2022-2023"

docker buildx build \
    --tag $REPO_NAME \
    --cache-from=type=registry,ref=${REPO_NAME}:cache \
    $@ \
    $DOCKER_DIR
