#!/bin/bash

# Környezeti beállítások
# Automatically detect and set both ROS_MASTER_URI and ROS_IP to the machine's IP address
MACHINE_IP=$(hostname -I | awk '{print $1}')
export ROS_MASTER_URI=http://$MACHINE_IP:11311
export ROS_IP=$MACHINE_IP

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
roslaunch -v voros_dome custom_world.launch  --screen

rqt_graph