# Robot Teleoperation

Robust outdoor teleoperation to increase efficiency and reduce costs based on ROS

[![Build Status](https://travis-ci.org/sachinkum0009/robot-teleoperation.svg?branch=master)](https://travis-ci.org/sachinkum0009/robot-teleoperation)

### To Do
- [x] Step to run <b>Robot</b>
- [x] Setup web server
- [x] Publish Image Data to web browser
- [x] Publish topic through web page
- [x] GUI for the robot
- [x] Unit Test Added
- [ ] Waypoint Navigation with GPS
- [x] Real-time GPS location
- [ ] Draw polygon on the map 

## Steps

## Test Result 
Publishing the image stream to the web

![Test Results](./assets/test01.png "Test Results")


## Steps to add GPS plugin to the robot
- install hector gps plugin
```
sudo apt-get install ros-melodic-hector-gazebo-plugins
'''
- go to the turtlebot3_description pkg 
- find the turtlebot3_waffle_pi.gazebo.xacro
- add the following code between the robot tag
```
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
```

## Two TurtleBot3 Robots
Steps to launch the two husky robot
- got to gui folder of robot_gui_bridge and run this command
```
roslaunch turtle_pkg main.launch
```
- roslaunch turtle_pkg two_turtlebot3.launch
- roslaunch robot_gui_bridge websocket.launch
- now you can open the browser and go to the url
```
localhost:8000
```
- find the robot_teleoperation.html file from there and you are ready to go.
=======

<hr>

## Test Result 
Publishing the image stream to the web

![Test Results](./assets/test01.png "Test Results")


## UI Changes
![UI for web interface](./assets/test02.png "UI for web interface")
