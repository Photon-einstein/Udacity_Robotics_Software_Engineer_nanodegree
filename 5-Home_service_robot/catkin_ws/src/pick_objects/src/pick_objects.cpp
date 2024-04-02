#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// pick up and drop off coordinates
double pickup_x = 0.0;
double pickup_y = -3.0;
double dropOff_x = 0.0;
double dropOff_y = 0.0; 

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true), ac2("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal pickUp;

  // set up the frame parameters of goal
  pickUp.target_pose.header.frame_id = "map";
  pickUp.target_pose.header.stamp = ros::Time::now();
  
  // Define a position and orientation for the robot to reach
  pickUp.target_pose.pose.position.x = pickup_x;
  pickUp.target_pose.pose.position.y = pickup_y;
  pickUp.target_pose.pose.position.z = 0.0;
  pickUp.target_pose.pose.orientation.w = -1.0;


   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pick up goal");
  ac.sendGoal(pickUp);
  
  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its pickUp position
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  	ROS_INFO("Robot reached his pick up position");
  else
  	ROS_INFO("The base failed to reach the pick up position");

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
	
	ros::Duration(5).sleep();

  	// Define a position and orientation for the robot to reach
	pickUp.target_pose.pose.position.x = dropOff_x;
  	pickUp.target_pose.pose.position.y = dropOff_y;
 	
	// Send the goal position and orientation for the robot to reach
 	ROS_INFO("Sending drop off goal with x= %.2f and y= %.2f", pickUp.target_pose.pose.position.x, pickUp.target_pose.pose.position.y);
 	ac2.sendGoal(pickUp);

	// Wait an infinite time for the results
  	ac2.waitForResult();

  	// Check if the robot reached its drop off position
  	if(ac2.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    		ROS_INFO("Robot reached his drop off position");
  	else
    		ROS_INFO("The base failed to reach the drop off position");
  
  }
  // Wait a few seconds before the terminal window disappears
  ros::Duration(10.0).sleep();
  return 0;
}
