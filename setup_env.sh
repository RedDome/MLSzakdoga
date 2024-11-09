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
          <visualize>0</visualize>
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
      <static>0</static>
    </model>
    <model name='box1'>
      <static>1</static>
      <link name='link'>
        <pose>2 2 0.5 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>1 1 1</size>
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
        <visual name='visual'>
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0 0 1</ambient>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <model name='unit_box'>
      <pose>-1.67576 -1.30188 0.5 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>1 1 1</size>
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
        <visual name='visual'>
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <include>
      <uri>model://turtlebot3_burger</uri>
    </include>
    <state world_name='custom_world'>
      <sim_time>118 795000000</sim_time>
      <real_time>119 260344844</real_time>
      <wall_time>1731132249 843635019</wall_time>
      <iterations>118795</iterations>
      <model name='box1'>
        <pose>0 0 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>2 2 0.5 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='ground_plane'>
        <pose>0 0 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0 0 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='turtlebot3_burger'>
        <pose>0.021239 -0.012728 0.010013 -0.000191 -0.012534 0.052565</pose>
        <scale>1 1 1</scale>
        <link name='base'>
          <pose>0.021239 -0.012728 0.010013 -0.000191 -0.012534 0.052565</pose>
          <velocity>0.000316 -0.000481 -0.001071 -0.024077 -0.010988 0.000722</velocity>
          <acceleration>-1.32292 3.00737 -2.7809 -1.03028 -0.015716 2.20803</acceleration>
          <wrench>-1.32292 3.00737 -2.7809 0 -0 0</wrench>
        </link>
        <link name='left_wheel'>
          <pose>0.009032 0.146797 0.010063 -0.000802 0.012555 -3.04141</pose>
          <velocity>6.8e-05 -0.000367 -0.005228 -0.023592 -0.009265 0.00088</velocity>
          <acceleration>-0.382364 0.959008 -6.55522 3.08221 0.31868 -0.612802</acceleration>
          <wrench>-0.038236 0.095901 -0.655522 0 -0 0</wrench>
        </link>
        <link name='lidar'>
          <pose>0.021239 -0.012727 0.010013 -0.000189 -0.012539 0.052565</pose>
          <velocity>0.000177 -0.000473 -0.001105 -0.023898 -0.009652 0.000809</velocity>
          <acceleration>-3.17376 7.33979 -2.57974 2.46923 -0.454344 -0.759993</acceleration>
          <wrench>-0.39672 0.917474 -0.322468 0 -0 0</wrench>
        </link>
        <link name='right_wheel'>
          <pose>0.052765 -0.167764 0.010211 0.002271 0.012171 -2.7929</pose>
          <velocity>-6e-06 0.000234 0.002889 -0.02196 -0.008701 0.000906</velocity>
          <acceleration>-1.15904 1.99729 1.99645 0.062385 1.44696 -0.570637</acceleration>
          <wrench>-0.115904 0.199729 0.199645 0 -0 0</wrench>
        </link>
      </model>
      <model name='unit_box'>
        <pose>-1.67576 -1.30188 0.5 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-1.67576 -1.30188 0.5 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>-0.004709 -9.78112 9.78158 0.712677 -0.009414 -4.3e-05</acceleration>
          <wrench>-0.004709 -9.78112 9.78158 0 -0 0</wrench>
        </link>
      </model>
      <light name='sun'>
        <pose>0 0 10 0 -0 0</pose>
      </light>
    </state>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>5.63025 -4.46437 1.76346 0 0.275643 2.35619</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
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
