# Használjuk a hivatalos ROS Noetic képet, amely tartalmazza a Gazebo 11-et
FROM osrf/ros:noetic-desktop-full

# Környezeti változók beállítása
ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger
ENV DISPLAY=":0"

# Rendszer frissítése és szükséges csomagok telepítése
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-tk \
    python3-catkin-tools \
    ros-noetic-turtlebot3-gazebo \
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
    && rm -rf /var/lib/apt/lists/*

# Python csomagok telepítése
RUN pip3 install --upgrade pip
RUN pip3 install \
    stable-baselines3[extra] \
    gym \
    catkin_pkg \
    empy \
    rospkg

# Catkin workspaces létrehozása
RUN mkdir -p /workspaces/MLSzakdoga/catkin_ws/src
WORKDIR /workspaces/MLSzakdoga/catkin_ws

# Forrásfájlok bemásolása
COPY ./setup_env.sh /workspaces/MLSzakdoga/setup_env.sh
RUN chmod +x /workspaces/MLSzakdoga/setup_env.sh

# Környezet változók beállítása
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /workspaces/MLSzakdoga/catkin_ws/devel/setup.bash" >> /root/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc
RUN echo "export DISPLAY=:0" >> /root/.bashrc  # A DISPLAY változó hozzáadása a bashrc-hez

CMD ["bash"]
 
