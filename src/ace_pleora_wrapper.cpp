#include "ace_usb3.h"
#include <ros/ros.h>

int main(int argc,char **argv)
{
  ros::init(argc,argv,"ros_pleora");
  ros::NodeHandle nh;

  try
  {
    AceUSB3 camera;
    camera.run();
  }
  catch(const std::exception &e)
  {
    ROS_ERROR("%s: %s",nh.getNamespace().c_str(),e.what());
  }
  
  return 0;
}
