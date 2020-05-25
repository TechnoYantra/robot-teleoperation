# setup file to install dependencies

echo "Installing dependencies"
sudo apt install ros-melodic-pointcloud-to-laserscan
sudo apt-get install ros-kinetic-swri-transform-util
sudo -H apt-get install -y ros-melodic-roswww 
sudo -H apt-get install -y ros-melodic-web-video-server
sudo apt-get install ros-melodic-hector-gazebo-plugins
sudo apt install ros-melodic-dwa-local-planner


echo "installing pip dependencies"
python -m pip install geographiclib rospkg
python3 -m pip install rospkg catkin_pkg
python3 -m pip install matplotlib

echo "cloning github repositories"
cd ..
git clone https://github.com/danielsnider/gps_goal.git
git clone https://github.com/GT-RAIL/robot_pose_publisher.git
git clone https://github.com/sachinkum0009/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git

echo "changing the branch of turtlebot3 to melodic-devel"
cd turtlebot3
git checkout melodic-devel
cd ..

echo "done with the setup"

echo "now compiling the pkg's"

cd ..
rosdep install --from-paths src --ignore-src -y

catkin_make

. devel/setup.bash

echo "Ready to launch files"
echo "run cmd 'export TURTLEBOT3_MODEL=waffle_pi' and 'roslaunch turtle_pkg main.launch' to launch the robot"

