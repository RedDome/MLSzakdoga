#!/bin/bash
set -e
pwd
ls -lai
echo "Entrypoint started...."
/workspaces/MLSzakdoga/setup_env.sh
echo "env setup done."
source /opt/ros/noetic/setup.bash
source /workspaces/MLSzakdoga/catkin_ws/devel/setup.bash &
export TURTLEBOT3_MODEL=burger
export DISPLAY=:0
MACHINE_IP=$(hostname -I | awk '{print $1}')
export ROS_MASTER_URI=http://$MACHINE_IP:11311
export ROS_IP=$MACHINE_IP

sleep 10
if ! pgrep -x "roscore" > /dev/null
then
    sleep 5
    python3 /workspaces/MLSzakdoga/src/main.py 
else
    echo "roscore is already running."
fi

#roslaunch voros_dome custom_world.launch  --screen

