#!/bin/sh

# Author : Rick Ramirez
# California State Polytechnic University Pomona
# Script follows here:

## Be sure to "source devel/setup.bash" the working directory if need be.
echo "Please give your workspace a name.... Ex: catkin_ws"
read WORKSPACE_NAME 
echo "Generating, $WORKSPACE_NAME workspace........."

mkdir -p ~/$WORKSPACE_NAME/src

cd ~/$WORKSPACE_NAME/src

catkin_init_workspace

cd ..

catkin_make

echo "Which distribution of ROS are you developing on? melodic or noetic"
read ROS_DISTRO
echo "Generating, $ROS_DISTRO workspace........."

git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git

cd $HOME/catkin_ws

rosdep update

rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src

catkin_make

echo "source $HOME/$WORKSPACE_NAME/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

gnome-terminal -e "roslaunch ur_gazebo ur5.launch"
sleep 1s

gnome-terminal -e "roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true"
sleep 1s

gnome-terminal -e "roslaunch ur5_moveit_config moveit_rviz.launch config:=true"

# https://github.com/ros-industrial/universal_robot/tree/melodic-devel-staging
