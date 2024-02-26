## Localization segment
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

##Setup
Use the following commands to build the project:

1. Navigate to catkin workspace root directory: '''cd catkin/'''
2. Compile Catkin Workspace: catkin_make
3. Launch Robot in world: source devel/setup.bash && roslaunch my_robot world.launch
4. Launch AMCL package (new tab): source devel/setup.bash && roslaunch my_robot amcl.launch
