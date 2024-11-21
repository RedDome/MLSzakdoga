#!/bin/bash

# Stop on any error
set -e

# Define the image name
IMAGE_NAME="ros-mls-deployment"

# Build the Docker image
cd ..
docker build -t $IMAGE_NAME -f ./deploy/Dockerfile .

echo "Build completed successfully. Image created as $IMAGE_NAME."
