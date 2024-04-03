# Udacity_Robotics_Software_Engineer_nanodegree

## Summary
This course aims to educate more about localization and navigation principles pertaining to autonomous robots, 
using a commonly used platform called Robot Operating System (ROS). It covers topics ranging from Gazebo simulation, 
communcation between ROS nodes, localization using Extended Kalman Filter or Particle Filter, Simultaneous 
Localization & Mapping (SLAM) and A* Path Planning. 

All of these techniques culminates into a final project where a home service robot capable of localization and navigation 
is developed. Instructions for compiling each project is available on each project's directory.

**Note**: These projects are developed in Ubuntu 16.04, ROS Kinetic and Gazebo 7.16.1. Please clone the master branch for 
latest version of all projects. 

## 1 - Gazebo World Segment
Gazebo is a useful simulation tool that can be used with ROS to render robots in a simulated environment. 
It comes with a model and world editor, along with presets models, that can allow for quick prototyping of a physical environment.

The main principles taught in this segment are:

1. Using model editor tool to render a robot with specified links & joints
2. Using World editor tool to render an environment (e.g. a house)
3. Running plugins during the launch of the Gazebo platform

The output of the word created in gazebo is shown bellow, where a office was buided using models in the gazebo library, some 
utilities were added from the library into the office and two robots were build in Gazebo, using the model capabilities of the 
that program.

<img src="0-Media/1-Gazebo_word_program_running.gif" width="900" height="400" />

## 2- ROS segment

The Robot Operating System (ROS) serves as middleware intended for facilitating communication among various components of robots, 
alongside providing commonly utilized packages for robotic applications. Within this project, diverse communication models were 
utilized across different nodes of the robot to enable it to navigate towards a white ball upon detection.

At a higher level, the 2D camera node continually assesses the presence of the white ball and its orientation relative to the robot's 
trajectory. Should a white ball be detected, a service is invoked to instruct the drive node to maneuver towards the ball with 
designated linear and angular velocities. The drive node, upon receiving this service call, disseminates the motion directives to the 
wheel actuation node, enabling the robot's movement.

The primary concepts emphasized in this section encompass:
1. Utilization of Packages & Catkin Workspaces
2. Development of ROS nodes & communication models (Publisher-Subscriber, Service-Client)

<img src="0-Media/2-Go_chase_it_program_running.gif" width="900" height="400" />

## 3 - Localization segment
In the realm of localization, two prevailing principles reign: the Extended Kalman Filter (EKF) and Monte Carlo Localization (Particle Filter). 
Armed with a map of its surroundings, along with data from its sensors and motor controls, the robot can employ either of these doctrines to 
gauge its positional state. For this endeavor, I harnessed the prowess of the Adaptive Monte Carlo Package from ROS. Initially, the robot initializes 
with a mapped terrain, with particles of uniform probability scattered haphazardly around its vicinity (depicted as verdant arrows). 
As the robot traverses, so do the particles. Each particle is then appraised with a likelihood measure, signifying its probable position and orientation, 
gauged by laser distance readings and the distances between its own position and landmarks on the map. The more substantial the likelihood, 
the greater the chance a particle will endure the resampling phase. Following multiple iterations of movement, one can discern the convergence of the 
green arrows towards the true location of the robot, an emblem of precise localization.

The crux of this segment elucidates the following algorithms:
- Extended Kalman Filter
- Adaptive Monte Carlo Localization (Particle Filter)

<img src="0-Media/3-Where_am_I_program _running.gif" width="900" height="400" />

## 4 - Mapping & SLAM Segment

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

<img src="0-Media/4-Map_my_world_program_running.gif" width="900" height="400" />

## 5- Path Planning & Navigation Segment

In the Path Planning & Navigation segment, we explore different approaches to path planning algorithms. 
These approaches include discrete, sample-based, and probabilistic methods. Each approach breaks down 
the configuration space into maps and plans paths in a different way.

In our project, we use a ROS package called gmapping to pre-map the environment. This package is based 
on the 2D Grid-Based FastSLAM algorithm, which doesn't have loop closure capabilities. To localize the 
robot at any given time, we utilize the Adaptive Monte Carlo Localization package from ROS.

For the home service robot simulation, we rely on the ROS navigation stack, which utilizes the Dijkstra 
algorithm. This stack allows us to simulate pickup and dropoff at preset points. To quickly launch the 
simulation, you can run the home_service.sh script located in the src/shellscripts/ directory.

Throughout this segment, we focus on teaching the following principles:

1. Path Planning Algorithms: We explore different approaches to path planning, understanding how they break 
down the configuration space and plan paths.
2. Simulating pickup and dropoff: We combine all the techniques we've learned so far to simulate the process 
of picking up and dropping off objects.

<img src="0-Media/5-Home_service_robot_program_running.gif" width="900" height="400" />