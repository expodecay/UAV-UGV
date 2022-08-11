#!/bin/sh

# Author : Rick Ramirez
# California State Polytechnic University Pomona
# Script follows here:

echo "Please give your workspace a name.... Ex: catkin_ws"
read WORKSPACE_NAME 
echo "Generating, $WORKSPACE_NAME workspace........."

mkdir -p ~/$WORKSPACE_NAME/src

cd ~/$WORKSPACE_NAME/src

catkin_init_workspace

cd ..

catkin_make

source ~/catkin_ws/devel/setup.bash
