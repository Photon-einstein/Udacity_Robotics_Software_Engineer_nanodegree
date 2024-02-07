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

## Gazebo World Segment
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
