#include <ros/ros.h>
#include <greenseeker_driver/greenseeker.hpp>

using namespace greenseeker_driver;

// main
int main(int argc, char** argv)
{
  ros::init(argc, argv, "greenseeker_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("greenseeker_driver_node: starting - greenseeker_driver");

  GreenSeeker greenskr(nh, pnh);

  ros::spin();

  return 0;
}
