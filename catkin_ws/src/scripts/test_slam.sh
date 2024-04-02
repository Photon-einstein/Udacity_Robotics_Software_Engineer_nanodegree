#!/bin/sh

# Define workspace variable
path_catkin_ws="${PWD}/../../"

xterm -e "export ROS_IP=127.0.0.1" &
sleep 1

# Open the workspace, source and launch turtlebot_world.launch
xterm -e "cd ${path_catkin_ws} && export ROS_IP=127.0.0.1&& source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch" &

sleep 5

# Open the workspace, source and launch gmapping_demo.launch
xterm -e "cd ${path_catkin_ws} && export ROS_IP=127.0.0.1 && source devel/setup.bash && roslaunch turtlebot_gazebo gmapping_demo.launch" &

sleep 5

# Open the workspace, source and launch view_navigation.launch
xterm -e "cd ${path_catkin_ws} && export ROS_IP=127.0.0.1 && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5

# Open the workspace, source and launch keyboard_teleop.launch
xterm -e "cd ${path_catkin_ws} && export ROS_IP=127.0.0.1 && source devel/setup.bash && roslaunch turtlebot_teleop keyboard_teleop.launch"
