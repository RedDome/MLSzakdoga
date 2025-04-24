#!/bin/bash

set -e

IMAGE_NAME="mlszakdoga_deploy"

cd ..
docker build -t $IMAGE_NAME -f ./deploy/Dockerfile .

echo "Build completed successfully. Image created as $IMAGE_NAME."
