/*
 * velocity_controller.cpp
 *
 *  Created on: 12 июн. 2017 г.
 *      Author: aminin
 */

#include "velocity_controller.h"
#include <std_msgs/Float32.h>
#include <math.h>

namespace velocity_controller
{

Blocker VelocityController::check_point(const velocity_plan_point& point)
{
  if (point.len > trajectory.get_length() )
    return TrajectoryFinish;
  if ( point.vel > max_velocity )
    return MaxVelExceeded;
  if ( point.acc > max_acc )
    return MaxAccExceeded;
  double curvature = trajectory.get_curvature(point.len);
  if ( point.vel * point.vel * curvature )
    return MaxCentripetalAccExceeded;
  if ( point.acc * curvature > max_angular_acc )
    return MaxAngularAccExceeded;
  // TODO add checking against map

  return NoBlock;
}

velocity_plan_point VelocityController::generate_point(velocity_plan_point& point, double t)
{
  velocity_plan_point next;
  double t2 = t*t;
  double t3 = t2*t;
  next.len = point.len + t*point.vel + t2*point.acc/2.0 + t3*point.jerk;
  next.vel = point.vel + t*point.acc + t2*point.jerk/2.0;
  next.acc = point.acc + t*point.jerk;
  return point;
}

bool VelocityController::generate_points(int n, velocity_plan_t& plan, double jerk)
{
  const double dt2 = dt*dt;
  const double dt3 = dt2*dt;
  for (int i = 0; i<n; ++i)
  {
    velocity_plan_point& prev = plan.back();
    prev.jerk = jerk;
    velocity_plan_point point = generate_point(prev, dt);
    if (!check_point(point))
      return false;
    plan.push_back(point);
  }
  return true;
}

bool VelocityController::generate_points(double plan_time, velocity_plan_t& plan, double jerk,
                                         double& remind_time, velocity_plan_point& remind_point)
{
  plan_time += remind_time;
  if( plan_time > dt )
  {
    double left_time = dt - remind_time;
    velocity_plan_point first = generate_point(remind_point, left_time);
    plan.push_back(first);
    plan_time -= left_time;
    int n = ceil( plan_time / dt );
    if ( !generate_points(n, plan, jerk) )
      return false;
    plan_time -= dt*n;
    remind_point = plan.back();
  }
  remind_time = plan_time;
  remind_point = generate_point(remind_point, remind_time);
  return check_point(remind_point);
}


bool VelocityController::check_stop(velocity_plan_t& stop_plan)
{
  double v0 = stop_plan.front().vel;
  double a0 = stop_plan.front().acc;
  // time of deceleration with const acceleration
  double t0 = v0 / max_acc + a0*a0 / (2*jerk*max_acc) - max_acc / jerk;

  // maximum value of deceleration
  double a1 = max_dcc;
  if ( t0 < 0 )
  {
    t0 = 0;
    a1 = sqrt(v0*jerk + a0*a0/2);
  }

  // time of jerking to zero acc
  double ta = a0/jerk;
  // time of jerking to deceleration a1 and to stop
  double td = a1/jerk;

  velocity_plan_point remind_point = stop_plan.back();
  double remind_time = 0;
  if ( ! generate_points(ta+td, stop_plan, -jerk, remind_time, remind_point) )
    return false;

  if ( ! generate_points(t0, stop_plan, 0.0, remind_time, remind_point) )
    return false;

  if ( ! generate_points(td, stop_plan, jerk, remind_time, remind_point) )
    return false;

  assert( (fabs(remind_point.vel) < 0.01) && (fabs(remind_point.acc) < 0.01 ) );
  stop_plan.push_back(remind_point);

  return true;
}

bool VelocityController::check_longrun(const velocity_plan_t& test_plan)
{
  velocity_plan_t long_plan;
  long_plan.push_back(test_plan.back());

  double v0 = test_plan.front().vel;
  double a0 = test_plan.front().acc;
  // time of deceleration with const acceleration
  double t0 = v0 / max_acc + a0*a0 / (2*jerk*max_acc) - max_acc / jerk;
  velocity_plan_point remind_point = test_plan.back();
  double remind_time = 0;
  // time of jerking to zero acc
  double ta = a0/jerk;

  if ( ! generate_points(ta, long_plan, -jerk, remind_time, remind_point) )
    return false;

  if ( ! generate_points(t0, long_plan, 0.0, remind_time, remind_point) )
    return false;

  return true;
}


bool VelocityController::update_velocity_plan()
{
  if ( velocity_plan.empty() )
    velocity_plan.push_back( velocity_plan_point() );
  if (velocity_plan.front().acc < 0 && velocity_plan.front().jerk > 0.1)
  {
      //end of deceleration to stop
      return false;
  }
  velocity_plan_t new_plan;
  new_plan.push_back(velocity_plan.front());
  new_plan.back().jerk = jerk;
  velocity_plan_point new_point = generate_point(new_plan.back(), dt);
  if ( ! check_point(new_point) )
    return false;
  new_plan.push_back(new_point);
  if ( check_stop(new_plan) )
  {
    velocity_plan_t long_run_plan;
    long_run_plan.push_back(new_plan.front());
    if (check_longrun(long_run_plan))
      return false;
    return true;
  }
  velocity_plan = std::move(new_plan);

  return false;
}

void VelocityController::on_timer(const ros::TimerEvent& event)
{
  // TODO check current velocity
  update_velocity_plan();
  std_msgs::Float32 cmd;
  cmd.data = velocity_plan.front().vel;
  cmdvel_pub.publish(cmd);
  velocity_plan.pop_front();
}

void VelocityController::on_trajectory(const sensor_msgs::PointCloudConstPtr& msg)
{
  trajectory = PointTrajectory(msg->points);
}

void VelocityController::on_map(const nav_msgs::OccupancyGridConstPtr& msg)
{

}

void VelocityController::on_odo(const nav_msgs::OdometryConstPtr& msg)
{
  current_velocity = msg->twist.twist.linear.x;
}

VelocityController::VelocityController():
    nh("~"),
    dt( 1.0 / nh.param("rate", 10.0 ) ),
    timer ( nh.createTimer( ros::Duration( dt ), &VelocityController::on_timer, this ) ),
    traj_sub( nh.subscribe("trajectory", 1, &VelocityController::on_trajectory, this) ),
    map_sub( nh.subscribe("map", 1,  &VelocityController::on_map, this) ),
    odo_sub( nh.subscribe("odo", 1, &VelocityController::on_odo, this) ),
    cmdvel_pub( nh.advertise<std_msgs::Float32>("velocity", 10) ),
    max_velocity( nh.param("max_velocity", 8.0) ),
    max_acc( nh.param("max_acc", 1.0) ),
    max_dcc( nh.param("max_dcc", 1.0) ),
    jerk( nh.param("jerk", 2.0) ),
    chassys_width( nh.param("chassys_width", 2.0) ),
    chassys_length( nh.param("chassys_length", 3.0) ),
    max_centripetal_acc( nh.param("max_centripetal_acc", 3.0) ),
    max_angular_acc( nh.param("max_angular_acc", 3.0)),
    long_run_time( nh.param("long_run_Time", 5.0) )
{

}

VelocityController::~VelocityController()
{
  // TODO Auto-generated destructor stub
}

} /* namespace velocity_controller */
