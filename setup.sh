# setup file to install dependencies

echo "Installing dependencies"
sudo apt install ros-melodic-pointcloud-to-laserscan
sudo apt-get install ros-kinetic-swri-transform-util
sudo -H apt-get install -y ros-melodic-roswww 
sudo -H apt-get install -y ros-melodic-web-video-server
sudo apt-get install ros-melodic-hector-gazebo-plugins

echo "installing pip dependencies"
python -m pip install geographiclib rospkg
python3 -m pip install rospkg catkin_pkg
python3 -m pip install matplotlib

echo "cloning github repositories"
cd ..
git clone https://github.com/danielsnider/gps_goal.git
git clone https://github.com/GT-RAIL/robot_pose_publisher.git

echo "done with the setup"

