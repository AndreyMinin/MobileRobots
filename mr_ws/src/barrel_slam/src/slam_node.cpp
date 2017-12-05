#include <ros/ros.h>
#include "slam.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simple_controller");
  ros::spin();
  return 0;

}

