/*
 * Controller.cpp
 *
 *  Created on: 30 апр. 2017 г.
 *      Author: aminin
 */

#include "controller.h"

#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud.h>

namespace simple_controller
{


template <class T>
T clip(T val, T max)
{
  if ( val > max )
    return max;
  if ( val < -max)
    return -max;
  return val;
}

void Controller::update_trajectory_segment()
{
  current_segment_length = (*current_segment)->get_point_length(robot_x, robot_y);

  while( current_segment_length < 0.0 )
  {
    if (current_segment == trajectory.begin())
      current_segment = trajectory.end();
    --current_segment;
    current_segment_length = (*current_segment)->get_point_length(robot_x, robot_y);
  }
  while (current_segment_length > (*current_segment)->get_length())
  {
    ++current_segment;
    if (current_segment == trajectory.end())
      current_segment = trajectory.begin();
    current_segment_length = (*current_segment)->get_point_length(robot_x, robot_y);
  }
//  ROS_DEBUG_STREAM("current segment length "<<current_segment_length);
}

void Controller::on_timer(const ros::TimerEvent& event)
{

  update_trajectory_segment();

//  double error = cross_track_error();
  double error = -(*current_segment)->get_point_distance(robot_x, robot_y);

  double diff_err = error - last_error;
  last_error = error;
  if ( fabs(error) < max_antiwindup_error )
    error_integral += error;
  else
    error_integral = 0.0;

  double angular_cmd = p_factor * error +
                        d_factor * diff_err +
                        i_factor * error_integral;
  double curvature = angular_cmd / current_velocity;
  std_msgs::Float32 cmd;
  cmd.data = clip<double>(curvature, max_curvature);
  steer_pub.publish(cmd);


  publish_trajectory();
//  ROS_DEBUG_STREAM("angular_rate cmd = "<<angular_rate);
}

void Controller::on_pose(const nav_msgs::OdometryConstPtr& odom)
{

  robot_x = odom->pose.pose.position.x;
  robot_y = odom->pose.pose.position.y;
  robot_theta = 2*atan2(odom->pose.pose.orientation.z,
                        odom->pose.pose.orientation.w);

//  current_velocity = odom->twist.twist.linear.x;
//  ROS_DEBUG_STREAM("x = "<<robot_x<<" "<<" y = "<<robot_y<<" "<<robot_theta);

  //ROS_DEBUG_STREAM("truth vel = "<<odom->twist.twist.linear.x);
}

void Controller::on_odo(const nav_msgs::OdometryConstPtr& odom)
{
  current_velocity = odom->twist.twist.linear.x;
  //ROS_DEBUG_STREAM("odom vel = "<<current_velocity);
}

double Controller::cross_track_error()
{
  double error = 0.0;
  if (robot_y < radius)
  {
    double rx = robot_x;
    double ry = robot_y - radius;
    error = sqrt(rx*rx + ry*ry) - radius;
  }
  else if ( robot_y > cy)
  {
    double rx = robot_x;
    double ry = robot_y - cy;
    error = sqrt(rx*rx + ry*ry) - radius;
  }
  else if ( robot_x > 0 )
  {
    error = robot_x - radius;
  }
  else if ( robot_x < 0 )
  {
    error = -radius - robot_x;
  }

  std_msgs::Float32 err_msg;
  err_msg.data = error;
  err_pub.publish(err_msg);

  return error;
}

void Controller::get_segment(std::list<TrajPtr>::iterator&  traj_it, double& len)
{
  traj_it = trajectory.end();

  if (robot_y < radius )
  {
    if ( robot_x >= 0 )
    {
      traj_it = trajectory.begin();

    }
  }
}

void add_point(sensor_msgs::PointCloud& msg, const tf::Vector3& point)
{
  geometry_msgs::Point32 p;
  p.x = point.x();
  p.y = point.y();
  p.z = point.z();
  msg.points.push_back(p);
}

void Controller::publish_trajectory()
{
  //prepare pointcloud message
  sensor_msgs::PointCloud msg;
  msg.header.frame_id = "odom";
  msg.header.stamp = ros::Time::now();
  static int seq(0);
  msg.header.seq = seq++;

  int trajectory_points_quantity = traj_length / traj_dl  + 1;
  int points_left = trajectory_points_quantity;
  msg.points.reserve( trajectory_points_quantity );
  double publish_len = 0;
  Trajectory::iterator it = current_segment;
  double start_segment_length = current_segment_length;
//  ROS_DEBUG_STREAM("start from "<<start_segment_length);

  while ( points_left )
  {
    double segment_length = (*it)->get_length();
    //add points from the segment
    int segment_points_quantity = std::min<int>( points_left, floor((segment_length - start_segment_length)/traj_dl) );
//    ROS_DEBUG_STREAM("segment points "<<segment_points_quantity);
    for ( int i = 0; i<=segment_points_quantity; ++i)
    {
      add_point(msg, (*it)->get_point(start_segment_length + i * traj_dl) );
    }
    points_left -= segment_points_quantity;
    //switch to next segment
    if (points_left)
    {
      //start point for next segment
      start_segment_length += (segment_points_quantity + 1)* traj_dl - segment_length;
      //ROS_DEBUG_STREAM("start segment length = "<<start_segment_length);
      ++it;
      if ( it == trajectory.end())
        it = trajectory.begin();
    }

  }

  traj_pub.publish(msg);
}

void Controller::reset()
{
  error_integral = 0.0;
  last_error = cross_track_error();
}

void Controller::reset(double p, double d, double i)
{
  reset();
  p_factor = p;
  d_factor = d;
  i_factor = i;
}


/*!
 * \brief constructor
 * loads parameters from ns
 * proportional, differential , integral - pid factors
 * max_antiwindup_error - max error for using integral component
 * trajectory consists of two circle segments connected with two lines
 * first circle center is (0, radius), second circle center is (0, cy)
 * radius - radius of circular parts
 * cy - center of second circle
 * traj_dl - discrete of published trajectory
 * traj_length - length of published trajectory
 * timer_period - timer discrete
 */
Controller::Controller(const std::string& ns):
    nh("~/" + ns),
    p_factor(nh.param("proportional", 1.0)),
    d_factor(nh.param("differential", 0.0)),
    i_factor(nh.param("integral", 0.0)),
    max_antiwindup_error( nh.param("max_antiwindup_error", 0.5) ),
    error_integral(0.0),
    last_error(0.0),
    radius( nh.param("radius", 10.0) ),
    cy( nh.param("cy", 2*radius) ), //default circle is in center (0,radius)
    max_curvature( nh.param("max_curvature", 0.2 ) ),
    traj_dl( nh.param("traj_dl", 0.2) ),
    traj_length( nh.param("traj_length", 5.0) ),
    pose_sub(nh.subscribe("ground_truth", 1, &Controller::on_pose, this)),
    timer( nh.createTimer( ros::Duration(nh.param("timer_period", 0.1)), &Controller::on_timer, this ) ),
    err_pub( nh.advertise<std_msgs::Float32>("error", 10) ),
    steer_pub( nh.advertise<std_msgs::Float32>("/steering", 10) ),
    odo_sub( nh.subscribe("odom", 1, &Controller::on_odo, this)),
    traj_pub( nh.advertise<sensor_msgs::PointCloud>("trajectory", 1) )
{
  //counter clock
  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( 1.0 / radius,    0,       0,    1.0,   0,   M_PI/2*radius) );
  trajectory.emplace_back( std::make_shared<trajectory::LinearTrajectory>  (        radius, radius, 0.0,   1.0,  cy - radius) );
  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( 1.0 / radius,   radius,   cy,   0.0,   1.0, M_PI/2*radius ) );
  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( 1.0 / radius,   0, radius + cy,   -1.0, 0.0, M_PI/2*radius ) );
  trajectory.emplace_back( std::make_shared<trajectory::LinearTrajectory>  (         -radius, cy,   0.0,   -1.0, cy - radius) );
  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( 1.0/ radius,   -radius, radius, 0.0,  -1.0,  M_PI/2*radius) );

  //clock wise track
//  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0 / radius,    0,       0,    1.0,   0,   M_PI/2*radius) );
//  trajectory.emplace_back( std::make_shared<trajectory::LinearTrajectory>  (        radius, -radius, 0.0,   -1.0,  cy - radius) );
//  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0 / radius,   radius,   -cy,   0.0,   -1.0, M_PI/2*radius ) );
//  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0 / radius,   0, -radius - cy,   -1.0,  0.0, M_PI/2*radius ) );
//  trajectory.emplace_back( std::make_shared<trajectory::LinearTrajectory>  (       -radius, -cy,   0.0,   1.0, cy - radius) );
//  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0/ radius,   -radius, -radius, 0.0,  1.0,  M_PI/2*radius) );


  current_segment = trajectory.begin();
}


Controller::~Controller()
{
  // TODO Auto-generated destructor stub
}

} /* namespace simple_controller */
