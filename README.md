# Robot Teleoperation

Robust outdoor teleoperation to increase efficiency and reduce costs based on ROS

[![Build Status](https://travis-ci.org/sachinkum0009/robot-teleoperation.svg?branch=master)](https://travis-ci.org/sachinkum0009/robot-teleoperation)

## Instructions

- clone the pkg

```bash
git clone https://github.com/sachinkum0009/robot-teleoperation.git
cd robot-teleoperation
git checkout robot
chmod +x setup.sh
./setup.bash

cd ../..
rosdep install --from-paths src --ignore-src -y


```

- add the xacro file in the turtle_pkg/urdf to the turtlebot3_description/urdf (copy pase or replace)

```xml
<?xml version="1.0"?>
<robot name="turtlebot3_waffle_pi_sim" xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:arg name="laser_visual"  default="false"/>
  <xacro:arg name="camera_visual" default="false"/>
  <xacro:arg name="imu_visual"    default="false"/>

  <gazebo reference="base_link">
    <material>Gazebo/DarkGrey</material>
  </gazebo>


 <!-- Adding the kinect sensor -->
<xacro:include filename="$(find kinect_v2)/urdf/kinect_v2.urdf.xacro" />

  <xacro:kinect_v2  parent="base_link">
    <origin xyz="-0.05 0 0.1" rpy="0 0 0"/>
 </xacro:kinect_v2>

 <gazebo>
    <plugin name="gps_controller" filename="libhector_gazebo_ros_gps.so">
      <alwayson>true</alwayson>
      <updaterate>1.0</updaterate>
      <bodyname>base_link</bodyname>
      <topicname>/fix</topicname>
      <velocitytopicname>/fix_velocity</velocitytopicname>
      <drift>5.0 5.0 5.0</drift>
      <gaussiannoise>0.1 0.1 0.1</gaussiannoise>
      <velocitydrift>0 0 0</velocitydrift>
      <velocitygaussiannoise>0.1 0.1 0.1</velocitygaussiannoise>
    </plugin>
  </gazebo>

  <gazebo reference="wheel_left_link">
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
    <kp>500000.0</kp>
    <kd>10.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>0.1</maxVel>
    <fdir1>1 0 0</fdir1>
    <material>Gazebo/FlatBlack</material>
  </gazebo>

  <gazebo reference="wheel_right_link">
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
    <kp>500000.0</kp>
    <kd>10.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>0.1</maxVel>
    <fdir1>1 0 0</fdir1>
    <material>Gazebo/FlatBlack</material>
  </gazebo>

  <gazebo reference="caster_back_right_link">
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>1.0</maxVel>
    <material>Gazebo/FlatBlack</material>
  </gazebo>

  <gazebo reference="caster_back_left_link">
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>1.0</maxVel>
    <material>Gazebo/FlatBlack</material>
  </gazebo>

  <gazebo reference="imu_link">
    <sensor type="imu" name="imu">
      <always_on>true</always_on>
      <visualize>$(arg imu_visual)</visualize>
    </sensor>
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo>
    <plugin name="turtlebot3_waffle_pi_controller" filename="libgazebo_ros_diff_drive.so">
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <odometrySource>world</odometrySource>
      <publishOdomTF>true</publishOdomTF>
      <robotBaseFrame>base_footprint</robotBaseFrame>
      <publishWheelTF>false</publishWheelTF>
      <publishTf>true</publishTf>
      <publishWheelJointState>true</publishWheelJointState>
      <legacyMode>false</legacyMode>
      <updateRate>30</updateRate>
      <leftJoint>wheel_left_joint</leftJoint>
      <rightJoint>wheel_right_joint</rightJoint>
      <wheelSeparation>0.287</wheelSeparation>
      <wheelDiameter>0.066</wheelDiameter>
      <wheelAcceleration>1</wheelAcceleration>
      <wheelTorque>10</wheelTorque>
      <rosDebugLevel>na</rosDebugLevel>
    </plugin>
  </gazebo>

  <gazebo>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <alwaysOn>true</alwaysOn>
      <bodyName>imu_link</bodyName>
      <frameName>imu_link</frameName>
      <topicName>imu</topicName>
      <serviceName>imu_service</serviceName>
      <gaussianNoise>0.0</gaussianNoise>
      <updateRate>200</updateRate>
      <imu>
        <noise>
          <type>gaussian</type>
          <rate>
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </rate>
          <accel>
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </accel>
        </noise>
      </imu>
    </plugin>
  </gazebo>



<!--link : https://www.raspberrypi.org/documentation/hardware/camera/-->
  <gazebo reference="camera_rgb_frame">
    <sensor type="camera" name="Pi Camera">
      <always_on>true</always_on>
      <visualize>$(arg camera_visual)</visualize>
      <camera>
          <horizontal_fov>1.085595</horizontal_fov>
          <image>
              <width>640</width>
              <height>480</height>
              <format>R8G8B8</format>
          </image>
          <clip>
              <near>0.03</near>
              <far>100</far>
          </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>30.0</updateRate>
        <cameraName>camera</cameraName>
        <frameName>camera_rgb_optical_frame</frameName>
        <imageTopicName>rgb/image_raw</imageTopicName>
        <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

## Install Dependencies

- Install following pkgs

```bash
sudo apt install ros-melodic-pointcloud-to-laserscan
```
<<<<<<< HEAD

- install the pkgs

```bash
sudo apt-get install ros-kinetic-gps-goal ros-kinetic-swri-transform-util

=======
- install the pkgs 
```
# sudo apt-get install ros-kinetic-gps-goal
# above pkg is included in the repository for melodic
sudo apt-get install ros-kinetic-swri-transform-util
sudo -H apt-get install -y ros-melodic-roswww 
sudo -H apt-get install -y ros-melodic-web-video-server
>>>>>>> d37d62a7c006e68d37da22609aff3874ad57b637
```

- install hector gps plugin

```bash

sudo apt-get install ros-melodic-hector-gazebo-plugins
```

- install the geographiclib pkg

```bash

pip install geographiclib rospkg

```

- install dependencies for python3

```bash

sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg
pip3 install matplotlib
```

- There are some other dependencies for rwt_nav pkg

## Steps

- run the cmd

```bash

export TURTLEBOT3_MODEL=waffle_pi
```

- launch the  file

```bash
roslaunch turtle_pkg main.launch
```

- after launching it if using conda

```bash
python grid_path_planning.py
```

- otherwise this line of the grid_path_planning.py

```bash

#! /usr/bin/env python
```

- now you can open the browser and go to the url

```bash
http://localhost:8000/rwt_nav/
```

### To Do

- [x] Step to run Robot
- [x] Setup web server
- [x] Publish Image Data to web browser
- [x] Publish topic through web page
- [x] GUI for the robot
- [x] Unit Test Added
- [x] Waypoint Navigation with GPS
- [x] Real-time GPS location
- [x] Draw polygon on the map
