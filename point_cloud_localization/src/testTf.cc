#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>

#include <cmath>        // std::abs

class testTF{
public:
  testTF(ros::NodeHandle nh):nh_(nh){
    _sub = nh_.subscribe("/odometry/filtered_map", 1 , &testTF::callback, this);
  }

  ~testTF(){};

  tf::TransformListener listener;
  tf::StampedTransform prevTransform;
  ros::Subscriber _sub;
  ros::NodeHandle nh_;


  void callback(const nav_msgs::Odometry::ConstPtr msg){
    ROS_INFO("HELLO");



    ros::Time t = msg->header.stamp;

    try
    {
      listener.waitForTransform("map","base_link", t, ros::Duration(5.0));
      listener.lookupTransform("map", "base_link", t, prevTransform);

      double curr_x = prevTransform.getOrigin().x();
      double curr_y = prevTransform.getOrigin().y();

      std::cout<<"current location is: "<<curr_x<<" , "<<curr_y<<std::endl;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ROS_INFO("ERRORORRRRRRR");
      std::cout<<"time is now: "<<ros::Time::now().toSec()<<std::endl;
      ros::Duration(1.0).sleep();
    }


  }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "filterRcInput");
  ros::NodeHandle n;

  testTF test(n);
  ros::spin();
  return 0;
}
