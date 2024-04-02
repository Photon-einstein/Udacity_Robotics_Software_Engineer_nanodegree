#!/bin/sh

# Define workspace variable
path_catkin_ws="${PWD}/../../"

# Open the workspace, source and launch turtlebot_world.launch
xterm -e "cd ${path_catkin_ws} && export ROS_IP=127.0.0.1 && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch" &

sleep 5

# Open the workspace, source and launch amcl_demo.launch
xterm -e "cd ${path_catkin_ws} && export ROS_IP=127.0.0.1 && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch map_file:=${path_catkin_ws}/src/map/map.yaml" &

sleep 5

# Open the workspace, source and launch view_navigation.launch
xterm -e "cd ${path_catkin_ws} && export ROS_IP=127.0.0.1 && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5

# Open the workspace, source and launch pick_objects pick_objects_demo
xterm -e "cd ${path_catkin_ws} && export ROS_IP=127.0.0.1 && source devel/setup.bash && rosrun add_markers add_markers" 

 
