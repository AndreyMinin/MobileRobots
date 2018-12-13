#include <ros/ros.h>
#include "slam.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simple_controller");
  Slam slam;
  ros::spin();
  return 0;

}

