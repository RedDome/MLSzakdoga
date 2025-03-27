#!/bin/bash

MACHINE_IP=$(hostname -I | awk '{print $1}')
export ROS_MASTER_URI=http://$MACHINE_IP:11311
export ROS_IP=$MACHINE_IP

source /opt/ros/noetic/setup.bash
source /workspaces/MLSzakdoga/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger

if ! pgrep -x "roscore" > /dev/null
then
    echo "Starting roscore..."
    roscore &
    sleep 5
else
    echo "roscore is already running."
fi

echo "Starting Gazebo with custom world..."
roslaunch -v voros_dome custom_world.launch  --screen