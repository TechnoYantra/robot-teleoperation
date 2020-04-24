rwt_nav
====================

Usage
-----
```
roslaunch rwt_nav rwt_nav.launch
```

To view live location of robot :
```
rosrun robot_pose_publisher robot_pose_publisher
```
- Current position of the robot is shown by yellow arrow.
- Ctrl + mouse movement = Zoom
- Shift + mouse movement = Pan

Launch the amcl node and move_base node.

Open your browser, and access to:

`http://<your host name>:8000/rwt_nav/`

for example : `http://localhost:8000/rwt_nav/`

Click on start job and mark four points on the map.

Run the following command in terminal to get the four coordinate points of the map.
```
rostopic echo /yug_pose
```

