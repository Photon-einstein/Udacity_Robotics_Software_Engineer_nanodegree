#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h> 
#include "geometry_msgs/PoseWithCovarianceStamped.h"

// pick up and drop off coordinates
double pickup_x = 0.0;
double pickup_y = -3.0;
double dropOff_x = 0.0;
double dropOff_y = 0.0; 

// pick up and drop off state variables
bool pickupStateReached = false;
bool dropOffStateReached = false;
bool pickupLog=false;
bool completeGoalReached = false;

double tolerance_distance = 1.0;

double distCalc(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2-x1, 2)+pow(y2-y1, 2));
}

void odometry_cb(const nav_msgs::Odometry &msg)
{
    double odom_x_coordinate = msg.pose.pose.position.x, odom_y_coordinate = msg.pose.pose.position.y;
    double distance_to_goal;
    // act on the given state
    if(pickupStateReached == false && dropOffStateReached == false) {
        distance_to_goal = distCalc(pickup_x, pickup_y, odom_x_coordinate, odom_y_coordinate);
        if(distance_to_goal <= tolerance_distance) {
            pickupStateReached = true;
            ROS_INFO("Robot reached the pickup zone.");
        }
    }

    if(pickupStateReached == true && dropOffStateReached == false) {
        distance_to_goal = distCalc(dropOff_x, dropOff_y, odom_x_coordinate, odom_y_coordinate);
        if(distance_to_goal <= tolerance_distance) {
            dropOffStateReached = true;
            ROS_INFO("Robot reached the drop off zone.");
        }
    }
}

void check_pose_state(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    double odom_x_coordinate = msg->pose.pose.position.x, odom_y_coordinate = msg->pose.pose.position.y;
    double distance_to_goal;
    // act on the given state
    if(pickupStateReached == false && dropOffStateReached == false) {
        distance_to_goal = distCalc(pickup_x, pickup_y, odom_x_coordinate, odom_y_coordinate);
        if(distance_to_goal <= tolerance_distance) {
            pickupStateReached = true;
            ROS_INFO("Robot reached the pickup zone.");
        }
        ROS_INFO("Robot at %f from the pick up position.", distance_to_goal);
    }

    if(pickupStateReached == true && dropOffStateReached == false) {
        distance_to_goal = distCalc(dropOff_x, dropOff_y, odom_x_coordinate, odom_y_coordinate);
        if(distance_to_goal <= tolerance_distance) {
            dropOffStateReached = true;
            ROS_INFO("Robot reached the pickup zone.");
        }
	ROS_INFO("Robot at %f from the drop off position.", distance_to_goal);
    }
}


int main( int argc, char** argv )
{
  ROS_INFO("Waiting for the system to come up.");
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub_amcl_pose = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, check_pose_state);
  //ros::Subscriber marker_sub = n.subscribe("odom", 10, odometry_cb);

  double simulation_pickup_duration = 5.0f;

  uint32_t shape = visualization_msgs::Marker::CUBE;
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "add_markers";
  marker.id = 0;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;   
 
  marker.pose.position.x = pickup_x;
  marker.pose.position.y = pickup_y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
    
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
    
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5f;
  marker.lifetime = ros::Duration();

  while (ros::ok()) {
        while (marker_pub.getNumSubscribers() < 1) {
            
            if (!ros::ok()) {
            return 0;
            }

            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }

        if(pickupStateReached == true && dropOffStateReached == false && pickupLog == false) {
            //marker.action = visualization_msgs::Marker::DELETE;
            ROS_INFO("The object was picked up by the robot, log in main.");
            ros::Duration(simulation_pickup_duration).sleep();
	    pickupLog = true;
            marker.pose.position.x = dropOff_x;
  	    marker.pose.position.y = dropOff_y;
        }

        if(pickupStateReached == true && dropOffStateReached == true && completeGoalReached == false) {
            marker.action = visualization_msgs::Marker::ADD;
            ROS_INFO("The object was drop off by the robot at the drop off location, log in main.");
            ros::Duration(simulation_pickup_duration).sleep();
            completeGoalReached = true;
        }
        marker_pub.publish(marker);
        ros::spinOnce();
  }
  return 0;
}
