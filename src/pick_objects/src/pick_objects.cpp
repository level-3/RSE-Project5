#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
std::map<std::string,float> map_pickup_position, map_pickup_pose;
std::map<std::string,float> map_dropoff_position, map_dropoff_pose;

int main(int argc, char** argv)
{
    // Initialize the simple_navigation_goals node
    ros::init(argc, argv, "pick_objects");
    ros::NodeHandle n;

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


    n.getParam("pickup_position", map_pickup_position);
    n.getParam("pickup_pose", map_pickup_pose);

    pickup_goal.target_pose.pose.position.x = map_pickup_position["x"];
    pickup_goal.target_pose.pose.position.y = map_pickup_position["y"];
    pickup_goal.target_pose.pose.position.z = map_pickup_position["z"];

    pickup_goal.target_pose.pose.orientation.x = map_pickup_pose["x"];
    pickup_goal.target_pose.pose.orientation.y = map_pickup_pose["y"];
    pickup_goal.target_pose.pose.orientation.z = map_pickup_pose["z"];
    pickup_goal.target_pose.pose.orientation.w = map_pickup_pose["w"];

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending pickup_goal");
    ac.sendGoal(pickup_goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Hooray, the base moved to pickup_goal");
        n.setParam("pickup_achieved", true);   
        ros::Duration(5.0).sleep() ;       
    }
    else
        ROS_INFO("The base failed to move for some reason");



//DROPOFF


    move_base_msgs::MoveBaseGoal dropoff_goal;

    // set up the frame parameters
    dropoff_goal.target_pose.header.frame_id = "map";
    dropoff_goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach


    n.getParam("dropoff_position", map_dropoff_position);
    n.getParam("dropoff_pose", map_dropoff_pose);

    dropoff_goal.target_pose.pose.position.x = map_dropoff_position["x"];
    dropoff_goal.target_pose.pose.position.y = map_dropoff_position["y"];
    dropoff_goal.target_pose.pose.position.z = map_dropoff_position["z"];

    dropoff_goal.target_pose.pose.orientation.x = map_dropoff_pose["x"];
    dropoff_goal.target_pose.pose.orientation.y = map_dropoff_pose["y"];
    dropoff_goal.target_pose.pose.orientation.z = map_dropoff_pose["z"];
    dropoff_goal.target_pose.pose.orientation.w = map_dropoff_pose["w"];



    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending dropoff_goal");
    ac.sendGoal(dropoff_goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Hooray, the base moved to dropoff_goal");
        n.setParam("dropoff_achieved", true);     
  
    }
    else
        ROS_INFO("The base failed to move for some reason");



    return 0;
}