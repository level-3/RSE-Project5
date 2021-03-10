#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


#include "services/Navigation.h"


ros::ServiceServer service;
ros::Publisher marker_pub;
visualization_msgs::Marker marker;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


bool add_navigation_marker(services::Navigation::Request  &req, services::Navigation::Response &res)
{
        //ROS_INFO("Navigation Goal Received");

        // Set shape type to arrow
        uint32_t shape = visualization_msgs::Marker::ARROW;


        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = 0;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = req.goal.position.x;
        marker.pose.position.y = req.goal.position.y;
        marker.pose.position.z = req.goal.position.z;
        marker.pose.orientation.x = req.goal.orientation.x;
        marker.pose.orientation.y = req.goal.orientation.y;
        marker.pose.orientation.z = req.goal.orientation.z;
        marker.pose.orientation.w = req.goal.orientation.w;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.5;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;

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
        res.msg = "Marker added successfully";  

        //ROS_INFO_STREAM(res.msg);

    return 1;
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "marker_server");
    ros::NodeHandle n;
    ros::Rate r(1);

    service = n.advertiseService("marker_server", add_navigation_marker);

    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    
    //ROS_INFO("Ready to receive markers");

    // Handle ROS communication events
    ros::spin();
}
