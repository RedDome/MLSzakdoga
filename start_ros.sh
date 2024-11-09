#!/bin/bash

# Környezeti beállítások
# start_ros.sh eleje
source /opt/ros/noetic/setup.bash
source /workspaces/MLSzakdoga/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger

# Ellenőrizzük, hogy a ROS_MASTER_URI nincs-e foglalva, majd indítjuk a roscore-t
if ! pgrep -x "roscore" > /dev/null
then
    echo "Starting roscore..."
    roscore &
    sleep 5  # Várunk, hogy a roscore teljesen elinduljon
else
    echo "roscore is already running."
fi

# Gazebo indítása a custom world fájllal
echo "Starting Gazebo with custom world..."
roslaunch voros_dome custom_world.launch
