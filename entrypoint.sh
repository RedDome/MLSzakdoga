# Entrypoint script with delay
#!/bin/bash
set -e

# Source the ROS and workspace setup
source /opt/ros/noetic/setup.bash
source /workspaces/MLSzakdoga/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
export DISPLAY=:0
# Automatically detect and set both ROS_MASTER_URI and ROS_IP to the machine's IP address
MACHINE_IP=$(hostname -I | awk '{print $1}')
export ROS_MASTER_URI=http://$MACHINE_IP:11311
export ROS_IP=$MACHINE_IP

# Run the setup scriptroswtf 
/workspaces/MLSzakdoga/setup_env.sh

# Add a short delay to allow services to start
sleep 10  # Adjust if necessary

# Launch the custom Gazebo world with TurtleBot3
roslaunch voros_dome custom_world.launch  --screen
