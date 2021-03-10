
#include <ros/ros.h>
#include <std_msgs/String.h>

class robot_controller
{
public:
  robot_controller()
  {
    //Topic you want to publish
    pub_ = n_.advertise<std_msgs::String>("/published_topic", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/subscribed_topic", 1, &robot_controller::callback, this);
  }

  void callback(const std_msgs::String& input)
  {
    std_msgs::String output;
    //.... do something with the input and generate the output...
    pub_.publish(output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "robot_controller");

  //Create an object of class SubscribeAndPublish that will take care of everything
  robot_controller robot_controllerObject;

  ros::spin();

  return 0;
}