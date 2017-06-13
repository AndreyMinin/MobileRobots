/*
 * velocity_controller.cpp
 *
 *  Created on: 12 июн. 2017 г.
 *      Author: aminin
 */

#include "velocity_controller.h"

namespace velocity_controller
{

void VelocityController::on_timer(const ros::TimerEvent& event)
{

}

void VelocityController::on_trajectory(const sensor_msgs::PointCloudConstPtr& msg)
{

}

VelocityController::VelocityController():
    nh("~"),
    traj_sub( nh.subscribe("trajectory", 1, &VelocityController::on_trajectory, this) ),
     timer ( nh.createTimer( ros::Duration(nh.param("timer_period", 0.1)), &VelocityController::on_timer, this ) )
{

}

VelocityController::~VelocityController()
{
  // TODO Auto-generated destructor stub
}

} /* namespace velocity_controller */
