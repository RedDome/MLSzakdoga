FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger
ENV DISPLAY=":0"

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-tk \
    python3-catkin-tools \
    ros-noetic-turtlebot3-gazebo \
    ros-noetic-turtlebot3-slam \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-controller-manager \
    ros-noetic-joint-state-controller \
    ros-noetic-effort-controllers \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-twist-mux \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-xacro \
    ros-noetic-diff-drive-controller \
    ros-noetic-robot-state-publisher \
    git \
    && rm -rf /var/lib/apt/lists/* 

RUN git config --global http.postBuffer 104857600
RUN git config --global http.lowSpeedLimit 0
RUN git config --global http.lowSpeedTime 999

RUN pip3 install --upgrade pip
RUN pip3 install --default-timeout=100 stable-baselines3[extra]
RUN pip3 install --default-timeout=100 gym
RUN pip3 install --default-timeout=100 catkin_pkg empy rospkg
RUN pip3 install --default-timeout=600 tensorflow
RUN pip3 install --default-timeout=600 loguru
RUN pip3 install --default-timeout=600 numpy
RUN pip3 install --default-timeout=600 pandas

RUN mkdir -p /workspaces/MLSzakdoga/catkin_ws/src
WORKDIR /workspaces/MLSzakdoga/catkin_ws/src
RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git && \
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

RUN apt-get update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -y
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /workspaces/MLSzakdoga/catkin_ws && catkin_make"

COPY ./setup_env.sh /workspaces/MLSzakdoga/setup_env.sh
RUN chmod +x /workspaces/MLSzakdoga/setup_env.sh

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /workspaces/MLSzakdoga/catkin_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc && \
    echo "export ROS_MASTER_URI=http://\$(hostname -I | awk '{print \$1}'):11311" >> /root/.bashrc && \
    echo "export ROS_IP=\$(hostname -I | awk '{print \$1}')" >> /root/.bashrc && \
    echo "export DISPLAY=:0" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

COPY setup_env.sh /setup_env.sh
RUN chmod +x /setup_env.sh

WORKDIR /workspaces/MLSzakdoga/catkin_ws

ENTRYPOINT ["/entrypoint.sh"]

