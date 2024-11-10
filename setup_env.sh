#!/bin/bash

echo "1. Ellenőrizzük, hogy a szükséges könyvtárak léteznek-e"
export CATKIN_WS_DIR="/workspaces/MLSzakdoga/catkin_ws"
mkdir -p "$CATKIN_WS_DIR/src/voros_dome/launch"
mkdir -p "$CATKIN_WS_DIR/src/voros_dome/worlds"
mkdir -p "$CATKIN_WS_DIR/src/voros_dome/rviz"

# Ellenőrzés
if [[ ! -d "$CATKIN_WS_DIR/src/voros_dome/launch" || ! -d "$CATKIN_WS_DIR/src/voros_dome/worlds" || ! -d "$CATKIN_WS_DIR/src/voros_dome/rviz" ]]; then
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
  <param name="use_sim_time" value="true"/>
  <param name="gazebo/start_timeout" value="60.0"/> <!-- Increase timeout if necessary -->
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

echo "5. A custom SDF world fájl létrehozása TurtleBot3 modellel és további akadályokkal"
cat <<EOF > "$CATKIN_WS_DIR/src/voros_dome/worlds/custom_world.sdf"
<sdf version='1.7'>
  <world name='custom_world'>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <contact>
              <collide_bitmask>65535</collide_bitmask>
              <ode/>
            </contact>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <model name='turtlebot3_burger'>
      <link name='base'>
        <inertial>
          <pose>-0.032 0 0.07 0 -0 0</pose>
          <inertia>
            <ixx>0.001</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.001</iyy>
            <iyz>0</iyz>
            <izz>0.001</izz>
          </inertia>
          <mass>1</mass>
        </inertial>
        <collision name='base_collision'>
          <pose>-0.032 0 0.07 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.14 0.14 0.14</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='base_visual'>
          <pose>-0.032 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://turtlebot3_common_meshes/burger_base.dae</uri>
              <scale>0.001 0.001 0.001</scale>
            </mesh>
          </geometry>
        </visual>
        <collision name='caster_collision'>
          <pose>-0.081 0 -0.004 0 -0 0</pose>
          <geometry>
            <sphere>
              <radius>0.005</radius>
            </sphere>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100000</mu>
                <mu2>100000</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='lidar'>
        <inertial>
          <pose>-0.02 0 0.161 0 -0 0</pose>
          <inertia>
            <ixx>0.001</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.001</iyy>
            <iyz>0</iyz>
            <izz>0.001</izz>
          </inertia>
          <mass>0.125</mass>
        </inertial>
        <collision name='lidar_sensor_collision'>
          <pose>-0.02 0 0.161 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.0508</radius>
              <length>0.055</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='lidar_sensor_visual'>
          <pose>-0.032 0 0.171 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://turtlebot3_common_meshes/lds.dae</uri>
              <scale>0.001 0.001 0.001</scale>
            </mesh>
          </geometry>
        </visual>
        <sensor name='hls_lfcd_lds' type='ray'>
          <always_on>1</always_on>
          <visualize>1</visualize>
          <pose>-0.032 0 0.171 0 -0 0</pose>
          <update_rate>1800</update_rate>
          <ray>
            <scan>
              <horizontal>
                <samples>360</samples>
                <resolution>1</resolution>
                <min_angle>0</min_angle>
                <max_angle>6.28</max_angle>
              </horizontal>
              <vertical>
                <samples>1</samples>
                <min_angle>0</min_angle>
                <max_angle>0</max_angle>
              </vertical>
            </scan>
            <range>
              <min>0.12</min>
              <max>3.5</max>
              <resolution>0.015</resolution>
            </range>
            <noise>
              <type>gaussian</type>
              <mean>0</mean>
              <stddev>0.01</stddev>
            </noise>
          </ray>
        </sensor>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='left_wheel'>
        <inertial>
          <pose>0 0.08 0.023 -1.57 0 0</pose>
          <inertia>
            <ixx>0.001</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.001</iyy>
            <iyz>0</iyz>
            <izz>0.001</izz>
          </inertia>
          <mass>0.1</mass>
        </inertial>
        <collision name='left_wheel_collision'>
          <pose>0 0.08 0.023 -1.57 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.033</radius>
              <length>0.018</length>
            </cylinder>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100000</mu>
                <mu2>100000</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='left_wheel_visual'>
          <pose>0 0.08 0.023 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://turtlebot3_common_meshes/tire.dae</uri>
              <scale>0.001 0.001 0.001</scale>
            </mesh>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='right_wheel'>
        <inertial>
          <pose>0 -0.08 0.023 -1.57 0 0</pose>
          <inertia>
            <ixx>0.001</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.001</iyy>
            <iyz>0</iyz>
            <izz>0.001</izz>
          </inertia>
          <mass>0.1</mass>
        </inertial>
        <collision name='right_wheel_collision'>
          <pose>0 -0.08 0.023 -1.57 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.033</radius>
              <length>0.018</length>
            </cylinder>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100000</mu>
                <mu2>100000</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='right_wheel_visual'>
          <pose>0 -0.08 0.023 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://turtlebot3_common_meshes/tire.dae</uri>
              <scale>0.001 0.001 0.001</scale>
            </mesh>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <joint name='left_wheel_joint' type='revolute'>
        <parent>base</parent>
        <child>left_wheel</child>
        <pose>0 0.08 0.023 -1.57 0 0</pose>
        <axis>
          <xyz>0 1 0</xyz>
          <limit>
            <lower>-1e+16</lower>
            <upper>1e+16</upper>
          </limit>
          <dynamics>
            <spring_reference>0</spring_reference>
            <spring_stiffness>0</spring_stiffness>
          </dynamics>
        </axis>
      </joint>
      <joint name='right_wheel_joint' type='revolute'>
        <parent>base</parent>
        <child>right_wheel</child>
        <pose>0 -0.08 0.023 -1.57 0 0</pose>
        <axis>
          <xyz>0 1 0</xyz>
          <limit>
            <lower>-1e+16</lower>
            <upper>1e+16</upper>
          </limit>
          <dynamics>
            <spring_reference>0</spring_reference>
            <spring_stiffness>0</spring_stiffness>
          </dynamics>
        </axis>
      </joint>
      <joint name='lidar_joint' type='fixed'>
        <parent>base</parent>
        <child>lidar</child>
        <pose>-0.032 0 0.171 0 -0 0</pose>
        <axis>
          <xyz expressed_in='__model__'>0 0 1</xyz>
          <limit>
            <lower>-1e+16</lower>
            <upper>1e+16</upper>
          </limit>
        </axis>
      </joint>
      <plugin name="turtlebot3_drive_controller" filename="libgazebo_ros_diff_drive.so">
        <ros>
          <namespace>/turtlebot3</namespace>
          <remapping>cmd_vel:=/cmd_vel</remapping>
          <remapping>odom:=/odom</remapping>
        </ros>
        <updateRate>50</updateRate>
        <leftJoint>left_wheel_joint</leftJoint>
        <rightJoint>right_wheel_joint</rightJoint>
        <wheelSeparation>0.16</wheelSeparation>
        <wheelDiameter>0.066</wheelDiameter>
        <torque>5.0</torque>
        <commandTopic>cmd_vel</commandTopic>
        <odometryTopic>odom</odometryTopic>
        <odometryFrame>odom</odometryFrame>
        <robotBaseFrame>base_footprint</robotBaseFrame>
        <useEncoders>true</useEncoders>
        <publishTf>true</publishTf>
      </plugin>
      <static>0</static>
    </model>

    <!-- Labirintus hozzáadása -->
    <model name='maze_walls'>
      <static>1</static>
      <link name='link'>
        <!-- Első fal -->
        <collision name='wall_1_collision'>
          <geometry>
            <box>
              <size>6 0.2 1</size>
            </box>
          </geometry>
          <pose>3 0 0.5 0 0 0</pose>
        </collision>
        <visual name='wall_1_visual'>
          <geometry>
            <box>
              <size>6 0.2 1</size>
            </box>
          </geometry>
          <pose>3 0 0.5 0 0 0</pose>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>

        <!-- Második fal -->
        <collision name='wall_2_collision'>
          <geometry>
            <box>
              <size>0.2 4 1</size>
            </box>
          </geometry>
          <pose>0 2 0.5 0 0 0</pose>
        </collision>
        <visual name='wall_2_visual'>
          <geometry>
            <box>
              <size>0.2 4 1</size>
            </box>
          </geometry>
          <pose>0 2 0.5 0 0 0</pose>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>

        <!-- Harmadik fal -->
        <collision name='wall_3_collision'>
          <geometry>
            <box>
              <size>6 0.2 1</size>
            </box>
          </geometry>
          <pose>-3 4 0.5 0 0 0</pose>
        </collision>
        <visual name='wall_3_visual'>
          <geometry>
            <box>
              <size>6 0.2 1</size>
            </box>
          </geometry>
          <pose>-3 4 0.5 0 0 0</pose>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
EOF

echo "6. A TurtleBot3 URDF fájl frissítése a szenzor pluginekkel és linkekkel"
TURTLEBOT3_URDF_FILE="/opt/ros/noetic/share/turtlebot3_description/urdf/turtlebot3_burger.urdf.xacro"
if grep -q "<plugin name=\"gazebo_ros_laser\"" "$TURTLEBOT3_URDF_FILE"; then
  echo "A szenzor pluginok már hozzá vannak adva."
else
  echo "Szenzor pluginok és linkek hozzáadása a TurtleBot3 URDF fájlhoz."
  sed -i '/<\/robot>/i \
  <link name="ultrasonic_sensor">\
    <visual>\
      <geometry>\
        <cylinder length="0.05" radius="0.01"/>\
      </geometry>\
    </visual>\
  </link>\
  <link name="infrared_sensor">\
    <visual>\
      <geometry>\
        <cylinder length="0.05" radius="0.01"/>\
      </geometry>\
    </visual>\
  </link>\
  <gazebo>\
    <plugin name="gazebo_ros_laser" filename="libgazebo_ros_laser.so">\
      <robotNamespace>/</robotNamespace>\
      <frameName>base_scan</frameName>\
      <topicName>/scan</topicName>\
    </plugin>\
    <plugin name="gazebo_ros_ultrasonic" filename="libgazebo_ros_range.so">\
      <robotNamespace>/</robotNamespace>\
      <frameName>ultrasonic_sensor</frameName>\
      <topicName>/ultrasonic_scan</topicName>\
      <radiationType>ultrasonic</radiationType>\
      <fov>0.05</fov>\
      <minRange>0.02</minRange>\
      <maxRange>3.0</maxRange>\
      <updateRate>10.0</updateRate>\
    </plugin>\
    <plugin name="gazebo_ros_infrared" filename="libgazebo_ros_range.so">\
      <robotNamespace>/</robotNamespace>\
      <frameName>infrared_sensor</frameName>\
      <topicName>/infrared_scan</topicName>\
      <radiationType>infrared</radiationType>\
      <fov>0.05</fov>\
      <minRange>0.01</minRange>\
      <maxRange>1.5</maxRange>\
      <updateRate>10.0</updateRate>\
    </plugin>\
  </gazebo>' "$TURTLEBOT3_URDF_FILE"
fi

echo "7. RViz konfiguráció létrehozása a szenzor adatok megjelenítéséhez"
cat <<EOF > "$CATKIN_WS_DIR/src/voros_dome/rviz/turtlebot3_display.rviz"
Panels:
  - Class: rviz/Displays
Displays:
  - Name: Robot Model
    Class: rviz/RobotModel
  - Name: LaserScan
    Class: rviz/LaserScan
    Topic: /scan
    Size (m): 0.1
  - Name: Ultrasonic
    Class: rviz/LaserScan
    Topic: /ultrasonic_scan
    Size (m): 0.1
  - Name: Infrared
    Class: rviz/LaserScan
    Topic: /infrared_scan
    Size (m): 0.1
EOF

echo "8. RViz launch fájl létrehozása"
cat <<EOF > "$CATKIN_WS_DIR/src/voros_dome/launch/rviz_display.launch"
<launch>
  <node name="rviz" pkg="rviz" type="rviz" args="-d $CATKIN_WS_DIR/src/voros_dome/rviz/turtlebot3_display.rviz" />
</launch>
EOF

echo "9. Buildeljük újra a catkin workspaces-t az új fájlokkal"
catkin_make
if [[ $? -ne 0 ]]; then
  echo "Hiba: A catkin workspace újrabuildelése sikertelen volt!"
  exit 1
fi

echo "10. Forrásoljuk az új workspace-t"
source "$CATKIN_WS_DIR/devel/setup.bash"

echo "Setup complete! You can now run Gazebo with the custom world and RViz by executing the following commands:"
echo "roslaunch voros_dome custom_world.launch"
echo "roslaunch voros_dome rviz_display.launch"
