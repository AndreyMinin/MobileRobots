/*
 * controller_node.cpp
 *
 *  Created on: 29 апр. 2017 г.
 *      Author: aminin
 */

#include <ros/ros.h>

#include "mpccontroller.h"

bool odo_received = false;
ros::Subscriber odo_sub;
void on_odo(const nav_msgs::OdometryConstPtr& odom)
{
  odo_received = true;
  odo_sub.shutdown();
}

void wait_gazebo(ros::NodeHandle& nh)
{
  //try to receive first message from robot before starting controller
  //this stuff was introduced for gazebo
  odo_sub = nh.subscribe("odom", 1, on_odo);
  while (ros::ok() && !odo_received)
  {
    ros::spinOnce();
  }
  ROS_DEBUG_STREAM("odo received");
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "mpc_controller");
  ros::NodeHandle nh("~/mpc_controller");

  //try to receive first message from robot before starting controller
  //this stuff was introduced for gazebo
  wait_gazebo(nh);
  mpc_controller::MPCController Cntrl;

  ros::spin();
  return 0;

}

