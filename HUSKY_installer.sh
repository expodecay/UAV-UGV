#!/bin/sh

# Author : Rick Ramirez
# California State Polytechnic University Pomona
# Script follows here:

echo "Which version of ROS are you running?"
read ROS_VERSION
echo "Installing HUSKY simulation for ROS, $ROS_VERSION........."


sudo apt-get update
sudo apt-get install ros-$ROS_VERSION-husky-desktop
sudo apt-get install ros-$ROS_VERSION-husky-simulator


gnome-terminal -- "roscore"
sleep 1s

gnome-terminal -e "roslaunch husky_gazebo husky_empty_world.launch"
sleep 1s

gnome-terminal -e "roslaunch husky_viz view_robot.launch"

