# Robot Teleoperation

Robust outdoor teleoperation to increase efficiency and reduce costs based on ROS

[![Build Status](https://travis-ci.org/sachinkum0009/robot-teleoperation.svg?branch=master)](https://travis-ci.org/sachinkum0009/robot-teleoperation)

## Instructions
- add the xacro file in the turtle_pkg/urdf to the turtlebot3_description/urdf (copy pase or replace)


## Install Dependencies
- Install following pkgs
```
sudo apt-get install ros-kinetic-gps-goal ros-kinetic-swri-transform-ut
```
- install the pkgs 
```
sudo apt-get install 
ros-kinetic-gps-goal ros-kinetic-swri-transform-util

```
- install hector gps plugin
```
sudo apt-get install ros-melodic-hector-gazebo-plugins
```
- install the geographiclib pkg
```
pip install geographiclib rospkg

```
- install dependencies for python3

```
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg
pip3 install matplotlib
```

- There are some other dependencies for rwt_nav pkg


## Steps
- run the cmd
```
export TURTLEBOT3_MODEL=waffle_pi
```


- launch the  file
```
roslaunch turtle_pkg gps_pc2sn_main.launch
```
- after launching it if using conda
```
python grid_path_planning.py
```
- otherwise this line of the grid_path_planning.py
```
#! /usr/bin/env python
```

- now you can open the browser and go to the url
```
http://localhost:8000/rwt_nav/
```


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
