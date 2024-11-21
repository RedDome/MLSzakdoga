#!/bin/bash

xhost local:docker

CONTAINER_NAME="ros-mls-container"
IMAGE_NAME="ros-mls-deployment"

ROS_IP=$(hostname -I | awk '{print $1}')
ROS_MASTER_URI="http://$ROS_IP:11311"
DISPLAY=":0"

docker run -d  \
    --name $CONTAINER_NAME \
    --network host \
    -e ROS_MASTER_URI=$ROS_MASTER_URI \
    -e ROS_IP=$ROS_IP \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    $IMAGE_NAME



