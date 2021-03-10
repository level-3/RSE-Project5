#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


#include "services/Navigation.h"


ros::ServiceClient marker_client;
ros::ServiceServer service;
ros::Publisher marker_pub;
visualization_msgs::Marker marker;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


bool add_nav_goal(services::Navigation::Request  &req, services::Navigation::Response &res)
{
    ROS_INFO("NAV_SERVER: Navigation Goal Received");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal pickup_goal;

    // set up the frame parameters
    pickup_goal.target_pose.header.frame_id = "map";
    pickup_goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    pickup_goal.target_pose.pose.position.x = req.goal.position.x;
    pickup_goal.target_pose.pose.position.y = req.goal.position.y;
    pickup_goal.target_pose.pose.position.z = req.goal.position.z;

    pickup_goal.target_pose.pose.orientation.x = req.goal.orientation.x;
    pickup_goal.target_pose.pose.orientation.y = req.goal.orientation.y;
    pickup_goal.target_pose.pose.orientation.z = req.goal.orientation.z;
    pickup_goal.target_pose.pose.orientation.w = req.goal.orientation.w;


    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending Pickup goal");

    ac.sendGoal(pickup_goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Pickup Goal Achieved");
        //ros::Duration(5.0).sleep() ;        
    }
    else
        ROS_INFO("The base failed to move to Pickup");

    res.msg = "Goal success";  

    return 1;
}



int main( int argc, char** argv )
{
    ros::init(argc, argv, "nav_goal_server");
    ros::NodeHandle n;
    ros::Rate r(1);

    service = n.advertiseService("nav_goal_server", add_nav_goal);
    
    //ROS_INFO("Readyfor goals");

    // Handle ROS communication events
    ros::spin();
}
