#include <ros/ros.h>

#include "planner.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simple_planner");
  ros::NodeHandle nh("~");
  simple_planner::Planner P(nh);

  ros::spin();

  return 0;
}
