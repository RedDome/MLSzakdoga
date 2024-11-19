# Use the ROS Noetic base image
FROM osrf/ros:noetic-desktop-full

# Set the working directory to /workspaces/MLSzakdoga
WORKDIR /workspaces/MLSzakdoga

# Copy necessary files from the development environment
COPY . /workspaces/MLSzakdoga

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-tk \
    ros-noetic-ros-base \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --upgrade pip
RUN pip3 install --default-timeout=100 stable-baselines3[extra]
RUN pip3 install --default-timeout=100 gym
RUN pip3 install --default-timeout=100 tensorflow

# ROS workspace setup
WORKDIR /workspaces/MLSzakdoga/catkin_ws
COPY ./catkin_ws /workspaces/MLSzakdoga/catkin_ws

# Set up the environment for ROS and custom variables
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /workspaces/MLSzakdoga/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc && \
    echo "export ROS_MASTER_URI=http://$(hostname -I | awk '{print $1}'):11311" >> ~/.bashrc && \
    echo "export ROS_IP=$(hostname -I | awk '{print $1}')" >> ~/.bashrc

# Ensure scripts are executable
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Define entrypoint for container startup
ENTRYPOINT ["/entrypoint.sh"]

# Expose the required ports for ROS communication
EXPOSE 11311

# Default command (could be overridden at runtime)
CMD ["/bin/bash"]
