#!/bin/bash

# Stop on any error
set -e

# Define container name and image
CONTAINER_NAME="ros-mls-container"
IMAGE_NAME="ros-mls-deployment"

# Set the necessary environment variables
ROS_IP=$(hostname -I | awk '{print $1}')
ROS_MASTER_URI="http://$ROS_IP:11311"
DISPLAY=":0"

# Run the Docker container with required options
docker run -it --rm \
    --name $CONTAINER_NAME \
    --network host \
    -e ROS_MASTER_URI=$ROS_MASTER_URI \
    -e ROS_IP=$ROS_IP \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd)/MLSzakdoga:/workspaces/MLSzakdoga \
    $IMAGE_NAME