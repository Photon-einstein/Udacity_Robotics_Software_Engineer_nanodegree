#!/bin/sh

# Define workspace variable
path_catkin_ws="${PWD}/../../"

xterm  -e  "cd ${path_catkin_ws} && export ROS_IP=127.0.0.1 & source /opt/ros/kinetic/setup.bash" &
sleep 1

# Launch Gazebo World & Robot
xterm  -e  "cd ${path_catkin_ws} && export ROS_IP=127.0.0.1 & roslaunch my_robot world.launch " &
sleep 5

# Launch gmapping node
xterm  -e  "cd ${path_catkin_ws} && export ROS_IP=127.0.0.1 & roslaunch my_robot gmapping.launch " &
sleep 5

# Launch teleops node
xterm  -e  "cd ${path_catkin_ws} && export ROS_IP=127.0.0.1 & rosrun teleop_twist_keyboard teleop_twist_keyboard.py" 
