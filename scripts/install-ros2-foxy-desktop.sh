#!/bin/sh

## Compiled script from various guides...
# - ROS2 foxy:  https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
# - Turtlebot3: https://web.archive.org/web/20240309202534/https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#turtlebot3-with-openmanipulator

## Ensure locale is UTF-8
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

## Enable required repositories
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

## Install Dependencies
# ROS2 - Desktop
sudo apt update && sudo apt upgrade -y
sudo apt install ros-foxy-desktop python3-argcomplete -y
# ROS2 - Bare Bones
# sudo apt install ros-foxy-ros-base python3-argcomplete -y
# ROS2 - Devtools
sudo apt install ros-dev-tools -y
# Activate ROS2 Environment
. /opt/ros/foxy/setup.sh
# TurtleBot3 Dependencies
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-foxy-gazebo-* \
  ros-foxy-cartographer \
  ros-foxy-cartographer-ros \
  ros-foxy-navigation2 \
  ros-foxy-nav2-bringup \
  ros-foxy-dynamixel-sdk \
  ros-foxy-ros2-control \
  ros-foxy-ros2-controllers \
  ros-foxy-gripper-controllers \
  ros-foxy-moveit
# TurtleBot3 Modules
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/
# git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
cd ~/turtlebot3_ws
colcon build --symlink-install

## Environment
sudo echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
sudo echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
sudo echo '. ~/turtlebot3_ws/install/setup.sh' >> ~/.bashrc
sudo echo '. /usr/share/gazebo/setup.sh' >> ~/.bashrc
sudo echo '. /opt/ros/foxy/setup.bash' >> ~/.bashrc