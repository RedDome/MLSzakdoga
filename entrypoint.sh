# Entrypoint script with delay
#!/bin/bash
set -e

# Source the ROS and workspace setup
source /opt/ros/noetic/setup.bash
source /workspaces/MLSzakdoga/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
export DISPLAY=:0
# Set ROS_MASTER_URI to default localhost
export ROS_MASTER_URI=http://localhost:11311
# Automatically detect and set ROS_IP to the machine's IP address
export ROS_IP=$(hostname -I | awk '{print $1}')

# Run the setup scriptroswtf 
/workspaces/MLSzakdoga/setup_env.sh

# Add a short delay to allow services to start
sleep 10  # Adjust if necessary

# Launch the custom Gazebo world with TurtleBot3
roslaunch voros_dome custom_world.launch  --screen
