#!/bin/bash

xhost local:docker

CONTAINER_NAME="mlszakdoga_deploy_container"
IMAGE_NAME="mlszakdoga_deploy"
WORKSPACE_FOLDER="$(cd "$(dirname "$0")/.." && pwd)"

if [ -z "$1" ]; then
  echo "Rossz futtatás! Megfelelő futattás: ./run.sh Learn / ./run.sh Continue / ./run.sh SaveData"
  exit 1
fi

FUNCTION_NAME=$1


ROS_IP=$(hostname -I | awk '{print $1}')
ROS_MASTER_URI="http://$ROS_IP:11311"
DISPLAY=":0"

docker run -d \
    --name $CONTAINER_NAME \
    --network host \
    --rm \
    -e ROS_MASTER_URI=$ROS_MASTER_URI \
    -e ROS_IP=$ROS_IP \
    -e DISPLAY=$DISPLAY \
    -e FUNCTION_NAME=$FUNCTION_NAME \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$WORKSPACE_FOLDER":/workspace \
    -w /workspace \
    $IMAGE_NAME



