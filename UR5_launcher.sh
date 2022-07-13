#!/bin/sh

# Author : Rick Ramirez
# California State Polytechnic University Pomona
# Script follows here:

gnome-terminal -- "roscore"
sleep 1s

gnome-terminal -e "roslaunch ur_gazebo ur5.launch"
sleep 1s

gnome-terminal -e "roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true"
sleep 1s

gnome-terminal -e "roslaunch ur5_moveit_config moveit_rviz.launch config:=true"
