# setup file to install dependencies

echo "Installing dependencies"
sudo apt install ros-melodic-pointcloud-to-laserscan
sudo apt-get install ros-kinetic-swri-transform-util
sudo -H apt-get install -y ros-melodic-roswww 
sudo -H apt-get install -y ros-melodic-web-video-server
sudo apt-get install ros-melodic-hector-gazebo-plugins
sudo apt install ros-melodic-dwa-local-planner
sudo apt install ros-melodic-turtlebot3*


echo "installing pip dependencies"
#python -m pip install geographiclib rospkg
#python3 -m pip install rospkg catkin_pkg
#python3 -m pip install matplotlib

echo "cloning github repositories"
cd ..
#git clone https://github.com/danielsnider/gps_goal.git
#git clone https://github.com/GT-RAIL/robot_pose_publisher.git

echo "changing urdf files"
echo $PWD
sudo cp robot-teleoperation/turtle_pkg/urdf/turtlebot3_waffle_pi.gazebo.xacro  /opt/ros/melodic/share/turtlebot3_description/urdf/turtlebot3_waffle_pi.gazebo.xacro

echo "done with the setup"

echo "now compiling the pkg's"

cd ..
rosdep install --from-paths src --ignore-src -y

catkin_make

. devel/setup.bash

echo "Ready to launch files"
echo "run cmd 'export TURTLEBOT3_MODEL=waffle_pi' and 'roslaunch turtle_pkg main.launch' to launch the robot"

