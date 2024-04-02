#!/bin/sh

# Define workspace variable
path_catkin_ws="${PWD}/../../"

# Open the workspace, source and launch turtlebot_world.launch
xterm -e "cd ${path_catkin_ws} && export ROS_IP=127.0.0.1 && source devel/setup.bash && roslaunch my_robot world.launch" &

sleep 5

# Open the workspace, source and launch amcl_demo.launch
xterm -e "cd ${path_catkin_ws} && export ROS_IP=127.0.0.1 && source devel/setup.bash && roslaunch my_robot amcl.launch" &

sleep 5

# Open the workspace, source and launch add_markers node
xterm -e "cd ${path_catkin_ws} && export ROS_IP=127.0.0.1 && source devel/setup.bash && roslaunch my_robot add_markers.launch" &

sleep 5

# Open the workspace, source and launch view_navigation.launch
xterm -e "cd ${path_catkin_ws} && export ROS_IP=127.0.0.1 && source devel/setup.bash && roslaunch my_robot pick_objects.launch"

