/*
 * laser2d_map.cpp
 *
 *  Created on: 25 июн. 2017 г.
 *      Author: aminin
 */

#include <ros/ros.h>
#include "laser_map.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "laser2d_map");
  ros::NodeHandle nh("~");
  laser2d_map::LaserMap map(nh);
  ros::spin();
  return 0;
}


