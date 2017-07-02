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

void VelocityController::on_map(const nav_msgs::OccupancyGridConstPtr& msg)
{

}

VelocityController::VelocityController():
    nh("~"),
    timer ( nh.createTimer( ros::Duration(nh.param("timer_period", 0.1)), &VelocityController::on_timer, this ) ),
    traj_sub( nh.subscribe("trajectory", 1, &VelocityController::on_trajectory, this) ),
    map_sub( nh.subscribe("map", 1,  &VelocityController::on_map, this) ),
    max_velocity( nh.param("max_velocity", 8.0) ),
    max_acc( nh.param("max_acc", 1.0) ),
    max_dcc( nh.param("max_dcc", 1.0) ),
    jerk( nh.param("jerk", 2.0) ),
    chassys_width( nh.param("chassys_width", 2.0) ),
    chassys_length( nh.param("chassys_length", 3.0) )
{

}

VelocityController::~VelocityController()
{
  // TODO Auto-generated destructor stub
}

} /* namespace velocity_controller */
