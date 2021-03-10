#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


#include "services/Navigation.h"

#include "nav_msgs/Odometry.h"

ros::ServiceServer service;
ros::Publisher marker_pub;

visualization_msgs::Marker marker;

ros::Subscriber odom_sub;


geometry_msgs::Point current_position;
geometry_msgs::Quaternion current_pose;



// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
uint32_t shape;



void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    
    current_pose = msg -> pose.pose.orientation;
    current_position = msg -> pose.pose.position;

    /*
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

    */

}


bool add_single_marker(geometry_msgs::Point marker_pos , geometry_msgs::Quaternion marker_pose, int type )
{

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
        marker.pose.position.x = marker_pos.x;
        marker.pose.position.y = marker_pos.y;
        marker.pose.position.z = marker_pos.z;

        marker.pose.orientation.x = marker_pose.x;
        marker.pose.orientation.y = marker_pose.y;
        marker.pose.orientation.z = marker_pose.z;
        marker.pose.orientation.w = marker_pose.w;
                // Set shape type to arrow

    if (type == 1)
    {
        shape = visualization_msgs::Marker::ARROW;
        marker.scale.x = 0.5;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;
    }
    else
    {

        shape = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;

    }
        // Set the marker type.  
        marker.type = shape;

        marker.lifetime = ros::Duration();

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
}


bool add_navigation_marker(services::Navigation::Request  &req, services::Navigation::Response &res)
{
        //ROS_INFO_STREAM(current_position);
        //ROS_INFO_STREAM(current_pose);

        //start point - PICKUP
        bool success_start = add_single_marker(current_position,current_pose,0);

        ros::Duration(5.0).sleep() ; 
        //start point - DROPOFF
        bool success_goal = add_single_marker(req.goal.position,req.goal.orientation,1);


        res.msg = "Markers added successfully";  

        //ROS_INFO_STREAM(res.msg);

    return 1;
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_marker");
    ros::NodeHandle n;
    ros::Rate r(1);

    service = n.advertiseService("add_marker", add_navigation_marker);

    odom_sub = n.subscribe("odom", 1000, odom_callback);
 
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    
    //ROS_INFO("Ready to receive markers");

    // Handle ROS communication events
    ros::spin();
}
