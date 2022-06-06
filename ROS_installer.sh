#!/bin/sh

# Author : Rick Ramirez
# California State Polytechnic University Pomona
# Script follows here:

echo "Which version of ROS do you need?"
read ROS_VERSION
echo "Installing ROS, $ROS_VERSION........."

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install ros-$ROS_VERSION-desktop-full

apt search ros-$ROS_VERSION

echo "source /opt/ros/$ROS_VERSION/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

sudo apt install python3-rosdep

sudo rosdep init

rosdep update

