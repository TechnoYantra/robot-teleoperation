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
