## Path Planning & Navigation Segment

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

## Setup
To run Home Service Robot Project:                      `cd ./catkin_ws/src/scripts && sh home_service_robot.sh`<br/>

## Other shellscripts used for testing:
1. Testing gmapping module:                             `cd ./catkin_ws/src/scripts && sh test_slam.sh`
2. Testing AMCL Localization & ROS Navigation Stack:    `cd ./catkin_ws/src/scripts && sh test_navigation.sh`
3. Testing pickup functionality:                        `cd ./catkin_ws/src/scripts && sh pickup_objects.sh`
4. Testing markers in ROS RViz:                         `cd ./catkin_ws/src/scripts && sh add_markers.sh`