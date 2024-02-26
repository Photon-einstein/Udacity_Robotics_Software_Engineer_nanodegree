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

![1-Gazebo_word_program_running](https://github.com/Photon-einstein/Udacity_Robotics_Software_Engineer_nanodegree/assets/31144077/da5a7709-314e-4232-a2c4-efb8d68067c3)

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

![2-Go_chase_it_program_running](https://github.com/Photon-einstein/Udacity_Robotics_Software_Engineer_nanodegree/assets/31144077/0c8939dd-e18f-4f93-93df-3cafb77c92e1)

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

![3-Where_am_I_program _running](https://github.com/Photon-einstein/Udacity_Robotics_Software_Engineer_nanodegree/assets/31144077/9be35363-3a0e-400b-8abe-685ed4e86920)
