# Mapping & SLAM Segment

In the domain of Mapping and SLAM, one prevalent mapping technique is Occupancy Grid Mapping. 
By leveraging sensor data and employing the Binary Bayes Filter, this method evaluates the 
probability of encountering obstacles within specific grid cells on the map. 
Successful mapping hinges on understanding the robot's initial position, its motor controls, 
and the data provided by its sensors.

**Simultaneous Localization and Mapping (SLAM)** amalgamate concepts from both localization and mapping. 
Through sensor data and motor control, the robot continuously builds a map of its environment, 
utilizing this map to determine its own position relative to it. The Online SLAM methodology furnishes 
the map and the robot's pose at a particular moment, while the Full SLAM methodology provides the map 
alongside all past poses of the robot. The primary techniques emphasized in this context include Grid-Based 
FastSLAM and GraphSLAM, which represent the Online and Full SLAM approaches, respectively. 
Notably, the Real Time Appearance Based Mapping constitutes part of the Online SLAM strategy in this project, 
employing a depth camera to enable 3D localization and mapping. Moreover, it facilitates loop closure, 
thereby recognizing previously visited locations to enhance map continuity.

This functionality is encapsulated in a ROS package (http://wiki.ros.org/rtabmap_ros). 
You can construct the rtab package from its source by adhering to the guidelines provided in the RTAB-github 
link (https://github.com/introlab/rtabmap_ros).

The main principles taught in this segment are: 
1) Occupancy Grid Mapping (Binary Bayes Filter)
2) Grid-Based FastSLAM
3) GraphSLAM
4) RTAB-map SLAM (Variant of GraphSLAM)

## Setup 
Use the following commands to build the project: 

1. Navigate to catkin workspace root directory: `cd catkin/`
2. Compile Catkin Workspace: `catkin_make`
3. Launch Robot in world: `source devel/setup.bash && roslaunch my_robot world.launch`
4. Launch Teleops package **(new tab)**: `source devel/setup.bash && rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
5. Launch RTAB mapping node **(new tab)**: `source devel/setup.bash && roslaunch my_robot mapping.launch`
6. To view rtab-map viewer **(new tab)**: `rtabmap-databaseViewer <PATH TO rtabmap.db>`<br/>

**Note:** rtabmap.db map database will be saved in ~/.ros directory. Run `roslaunch my_robot localization.launch` to instead in Step 5 for localization.
