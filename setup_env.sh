#!/bin/bash

echo "1. Ellenőrizzük, hogy a szükséges könyvtárak léteznek-e"
export CATKIN_WS_DIR="/workspaces/MLSzakdoga/catkin_ws"
mkdir -p "$CATKIN_WS_DIR/src/voros_dome/launch"
mkdir -p "$CATKIN_WS_DIR/src/voros_dome/worlds"

# Ellenőrzés
if [[ ! -d "$CATKIN_WS_DIR/src/voros_dome/launch" || ! -d "$CATKIN_WS_DIR/src/voros_dome/worlds" ]]; then
  echo "Hiba: A szükséges könyvtárak létrehozása nem sikerült!"
  exit 1
fi

echo "2. Inicializáljuk a catkin workspaces-t"
cd "$CATKIN_WS_DIR" || { echo "Hiba: Nem sikerült belépni a $CATKIN_WS_DIR könyvtárba"; exit 1; }
catkin_make
if [[ $? -ne 0 ]]; then
  echo "Hiba: A catkin workspace inicializálása sikertelen volt!"
  exit 1
fi

echo "3. Forrásoljuk a workspace-t a környezethez"
source /opt/ros/noetic/setup.bash
source "$CATKIN_WS_DIR/devel/setup.bash"
export TURTLEBOT3_MODEL=burger

# Ellenőrizzük, hogy a workspace forrásolása sikeres volt-e
if [[ -z "$ROS_PACKAGE_PATH" ]]; then
  echo "Hiba: A ROS környezet forrásolása nem sikerült!"
  exit 1
fi

echo "4. Launch fájl létrehozása a TurtleBot3 és custom world indításához"
cat <<EOF > "$CATKIN_WS_DIR/src/voros_dome/launch/custom_world.launch"
<launch>
  <arg name="world_name" default="$CATKIN_WS_DIR/src/voros_dome/worlds/custom_world.sdf"/>
  <include file="/opt/ros/noetic/share/gazebo_ros/launch/empty_world.launch">
    <arg name="world_name" value="\$(arg world_name)"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
  </include>
</launch>
EOF

# Ellenőrzés
if [[ ! -f "$CATKIN_WS_DIR/src/voros_dome/launch/custom_world.launch" ]]; then
  echo "Hiba: A custom_world.launch fájl létrehozása sikertelen volt!"
  exit 1
fi

echo "5. A custom SDF world fájl létrehozása akadályokkal és TurtleBot3 modellel"
cat <<EOF > "$CATKIN_WS_DIR/src/voros_dome/worlds/custom_world.sdf"
<?xml version="1.0"?>
<sdf version="1.5">
  <world name="custom_world">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://turtlebot3_burger</uri>
    </include>
    <model name="box1">
      <static>true</static>
      <link name="link">
        <pose>2 2 0.5 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.0 0.0 1</ambient>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
EOF

# Ellenőrzés
if [[ ! -f "$CATKIN_WS_DIR/src/voros_dome/worlds/custom_world.sdf" ]]; then
  echo "Hiba: A custom_world.sdf fájl létrehozása sikertelen volt!"
  exit 1
fi

echo "6. Buildeljük újra a catkin workspaces-t az új fájlokkal"
catkin_make
if [[ $? -ne 0 ]]; then
  echo "Hiba: A catkin workspace újrabuildelése sikertelen volt!"
  exit 1
fi

echo "7. Forrásoljuk az új workspace-t"
source "$CATKIN_WS_DIR/devel/setup.bash"

# Végső ellenőrzés, hogy minden készen áll
if [[ $? -ne 0 ]]; then
  echo "Hiba: Az új workspace forrásolása sikertelen volt!"
  exit 1
fi

echo "Setup complete! You can now run Gazebo with the custom world by executing the following command:"
echo "roslaunch voros_dome custom_world.launch"
