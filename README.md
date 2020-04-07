# Robot Teleoperation

Robust outdoor teleoperation to increase efficiency and reduce costs based on ROS

[![Build Status](https://travis-ci.org/sachinkum0009/robot-teleoperation.svg?branch=master)](https://travis-ci.org/sachinkum0009/robot-teleoperation)

### To Do
- [x] Step to run <b>Robot</b>
- [x] Setup web server
- [x] Publish Image Data to web browser
- [x] Publish topic through web page
- [x] GUI for the robot

## Steps
- Install the husky pkg (e.g. husky_gazebo huskty_simulator, etc.)
- Locate the husky_description pkg and add the following code in the decoration.urdf.xacro file
```

    <!-- Adding the camera -->
    <gazebo reference="top_plate_link">
    <sensor type="camera" name="camera">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>800</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <!-- Noise is sampled independently per pixel on each frame.
               That pixel's noise value is added to each of its color
               channels, which at that point lie in the range [0,1]. -->
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>husky/camera</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>huskty_rear_link</frameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>

```
- Launch the simulator
```
roslaunch husky_gazebo husky_empty_world.launch  
```
- Now launch the websocket launch file
```
roslaunch robot_gui_bridge websocket.launch
```
- After that go to the gui directory inside the robot_gui_bidge and enter the command
```
python3 -m http.server
```
- Now open the web browser and locate the following address
```
localhost:8000
```

<hr>

## Test Result 
Publishing the image stream to the web

![Test Results](./assets/test01.png "Test Results")


## Two TurtleBot3 Robots
Steps to launch the two husky robot
- got to gui folder of robot_gui_bridge and run this command
```
python3 -m http.server
```
- roslaunch turtle_pkg two_turtlebot3.launch
- roslaunch robot_gui_bridge websocket.launch
- now you can open the browser and go to the url
```
localhost:8000
```
- find the robot_teleoperation.html file from there and you are ready to go.

## UI Changes
![UI for web interface](./assets/test02.png "UI for web interface")
