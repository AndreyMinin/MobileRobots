/*
 * velocity_controller.h
 *
 *  Created on: 12 июн. 2017 г.
 *      Author: aminin
 */

#ifndef SRC_VELOCITY_CONTROLLER_H_
#define SRC_VELOCITY_CONTROLLER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>

namespace velocity_controller
{

class VelocityController
{
protected:
  ros::NodeHandle nh;
  ros::Timer timer;
  ros::Subscriber traj_sub;
  ros::Subscriber map_sub;

  double max_velocity;
  double max_acc;
  double max_dcc;
  double jerk;
  double chassys_width;
  double chassys_length;

  void on_timer(const ros::TimerEvent& event);
  void on_trajectory(const sensor_msgs::PointCloudConstPtr& msg);
  void on_map(const nav_msgs::OccupancyGridConstPtr& msg);
public:
  VelocityController();
  virtual ~VelocityController();
};

} /* namespace velocity_controller */

#endif /* SRC_VELOCITY_CONTROLLER_H_ */
