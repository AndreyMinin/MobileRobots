/*
 * main.cpp
 *
 *  Created on: 12 июн. 2017 г.
 *      Author: aminin
 */

/*
 * controller_node.cpp
 *
 *  Created on: 29 апр. 2017 г.
 *      Author: aminin
 */

#include <ros/ros.h>

#include "velocity_controller.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "vel_controller");
  ros::NodeHandle nh("~/velocity_controller");
  velocity_controller::VelocityController VCtl;
  ros::spin();
  return 0;

}


