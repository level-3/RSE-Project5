#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "nav_msgs/Odometry.h"

ros::ServiceServer service;
ros::Publisher marker_pub;

visualization_msgs::Marker marker;

ros::Subscriber odom_sub;

geometry_msgs::Point current_position;
geometry_msgs::Quaternion current_pose;

std::map<std::string,float> map_pickup_position, map_pickup_pose;
std::map<std::string,float> map_dropoff_position, map_dropoff_pose;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
uint32_t shape;






int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers_static");
    ros::NodeHandle n;
    ros::Rate r(1);

    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    n.setParam("pickup_achieved", false);
    n.setParam("dropoff_achieved", false);   

    map_pickup_position["x"] = 3.0;
    map_pickup_position["y"] = -1.0;
    map_pickup_position["z"] = 0;

    map_pickup_pose["x"] = 0;
    map_pickup_pose["y"] = 0;
    map_pickup_pose["z"] = 0;
    map_pickup_pose["w"] = 1;


    n.setParam("pickup_position", map_pickup_position);
    n.setParam("pickup_pose", map_pickup_pose);


    shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = map_pickup_position["x"];
    marker.pose.position.y = map_pickup_position["y"];
    marker.pose.position.z = map_pickup_position["z"];

    marker.pose.orientation.x = map_pickup_pose["x"];
    marker.pose.orientation.y = map_pickup_pose["y"];
    marker.pose.orientation.z = map_pickup_pose["z"];
    marker.pose.orientation.w = map_pickup_pose["w"];


    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    // Set the marker type.  
    marker.type = shape;

    marker.lifetime = ros::Duration(5);

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
        {
        if (!ros::ok())
            {
                ROS_WARN_ONCE("Subscribed");
            }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
        }

    marker_pub.publish(marker);

    ros::Duration(10.0).sleep() ;  


    map_dropoff_position["x"] = -3.0;
    map_dropoff_position["y"] = -3.0;
    map_dropoff_position["z"] = 0;

    map_dropoff_pose["x"] = 0;
    map_dropoff_pose["y"] = 0;
    map_dropoff_pose["z"] = 0;
    map_dropoff_pose["w"] = 1;


    n.setParam("dropoff_position", map_dropoff_position);
    n.setParam("dropoff_pose", map_dropoff_pose);


    shape = visualization_msgs::Marker::CUBE;
    // visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = map_dropoff_position["x"];
    marker.pose.position.y = map_dropoff_position["y"];
    marker.pose.position.z = map_dropoff_position["z"];

    marker.pose.orientation.x = map_dropoff_pose["x"];
    marker.pose.orientation.y = map_dropoff_pose["y"];
    marker.pose.orientation.z = map_dropoff_pose["z"];
    marker.pose.orientation.w = map_dropoff_pose["w"];


    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    // Set the marker type.  
    marker.type = shape;

    marker.lifetime = ros::Duration(5);

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
        {
        if (!ros::ok())
            {
                ROS_WARN_ONCE("Subscribed");
            }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
        }

    marker_pub.publish(marker);

    // Handle ROS communication events
    ros::spin();
}
