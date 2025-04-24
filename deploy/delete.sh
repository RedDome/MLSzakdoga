#!/bin/bash

IMAGE_NAME="mlszakdoga_deploy"

if docker image inspect "$IMAGE_NAME" > /dev/null 2>&1; then
    echo "Deleting Docker image: $IMAGE_NAME"
    docker rmi "$IMAGE_NAME"
else
    echo "Docker image $IMAGE_NAME not found."
fi
