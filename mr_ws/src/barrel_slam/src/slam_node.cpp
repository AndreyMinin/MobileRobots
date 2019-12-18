#include <ros/ros.h>
#include "slam.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "barrel_slam");
  Slam slam;
  ros::spin();
  return 0;

}

