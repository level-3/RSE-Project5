#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <std_msgs/String.h>

#include <geometry_msgs/Vector3.h>

#include "services/Navigation.h"

#include <iostream>

// #include <tf2/LinearMath/Quaternion.h>
#include <cmath>
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::ServiceClient marker_client;
ros::ServiceClient goal_client;


//https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles (modified for geometry_msg)
geometry_msgs::Quaternion  ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    geometry_msgs::Quaternion  q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}



bool call_services(double x,double y,double theta )
{

  services::Navigation nav_goal;
  services::Navigation nav_marker;

  geometry_msgs::Point pos;
  geometry_msgs::Quaternion orient;
  
  pos.x = x;  pos.y = y;  pos.z = 0.0;
  orient = ToQuaternion(theta,0,0);
  
  //ROS_INFO_STREAM(orient);

  nav_goal.request.goal.position = pos;
  nav_goal.request.goal.orientation = orient;
  
  nav_marker = nav_goal;

  marker_client.call(nav_goal);
  ROS_INFO_STREAM(nav_goal.response.msg);

  goal_client.call(nav_marker);
  ROS_INFO_STREAM(nav_marker.response.msg);

  return 1;
}

int main(int argc, char** argv){
  
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
    
  //ROS_INFO("Pick Object started");

  marker_client = n.serviceClient<services::Navigation>("add_marker");
  goal_client = n.serviceClient<services::Navigation>("nav_goal_server");


  //location x, y , theta (yaw)
  call_services(-2.0, -4.0, 0.0); //kitchen

  call_services(3.0, -7.0, M_PI);  //living room
  call_services(3.5, -0.5, M_PI);  //spare bedroom
  call_services(-3.0, 3.0, 0.0);  //bedroom
  call_services(3.0, 3.5, M_PI);   //office

  call_services(-2.0, -4.0, 0.0); //kitchen

  return 0;
}