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

namespace velocity_controller
{

class VelocityController
{
protected:
  ros::NodeHandle nh;
  ros::Timer timer;
  ros::Subscriber traj_sub;
  void on_timer(const ros::TimerEvent& event);
  void on_trajectory(const sensor_msgs::PointCloudConstPtr& msg);
public:
  VelocityController();
  virtual ~VelocityController();
};

} /* namespace velocity_controller */

#endif /* SRC_VELOCITY_CONTROLLER_H_ */
