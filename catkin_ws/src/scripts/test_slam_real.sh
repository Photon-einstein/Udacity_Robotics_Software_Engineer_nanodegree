#!/bin/sh

xterm  -e  " source /opt/ros/kinetic/setup.bash" &
xterm -e "export ROS_IP=127.0.0.1" &
sleep 1

# Launch Gazebo World & Robot
xterm  -e  " roslaunch my_robot world.launch " &
sleep 5

# Launch gmapping node
xterm  -e  " roslaunch my_robot gmapping.launch " &
sleep 5

# Launch teleops node
xterm  -e  " rosrun teleop_twist_keyboard teleop_twist_keyboard.py" 
