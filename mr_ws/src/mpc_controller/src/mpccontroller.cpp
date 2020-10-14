/*
 * Controller.cpp
 *
 *  Created on: 30 апр. 2017 г.
 *      Author: aminin
 */

#include "mpccontroller.h"

#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <tf/transform_datatypes.h>
namespace mpc_controller
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

void MPCController::update_trajectory_segment()
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

void MPCController::update_control_points() {
  control_points.resize(control_points_num);
  Trajectory::iterator segment = current_segment;
  tf::Vector3 pose(robot_x, robot_y, 0);
  ROS_DEBUG_STREAM("control points");
  double control_point_distance = (*segment)->get_point_length(pose.x(), pose.y());
  for(std::size_t i = 0; i<control_points_num; ++i) {
    control_point_distance += i*control_points_dl;
    while (control_point_distance > (*segment)->get_length()) {
      control_point_distance -= (*segment)->get_length();
      ++segment;
      if (segment == trajectory.end())
        segment = trajectory.begin();
    }
    control_points[i] = (*segment)->get_point(control_point_distance);
    ROS_DEBUG_STREAM(i<<": "<<control_points[i].x()<<" "<<control_points[i].y());
  }
}

void MPCController::convert_control_points() {
  tf::Transform world2robot = robot2world.inverse();
  ROS_DEBUG_STREAM("control points in robot coordinates ");
  for (auto& point: control_points) {
    point = world2robot(point);
    ROS_DEBUG_STREAM(point.x()<<" "<<point.y());
  }

}

// calculates polynom coefficients
void MPCController::calculate_control_coefs() {
  const int order = 3; // we have 4 coefficients
  assert(order <= control_points.size() - 1);
  Eigen::MatrixXd A(control_points.size(), order + 1);

  for (int i = 0; i<control_points.size(); ++i) {
    A(i,0) = 1.0;
  }
  Eigen::VectorXd yvals(control_points.size());
  for (int j = 0; j < control_points.size(); j++)
  {
    yvals(j) = control_points[j].y();
    for (int i = 0; i < order; i++)
    {
      A(j, i + 1) = A(j, i) * control_points[j].x();
    }
  }
  auto Q = A.householderQr();
  Eigen::VectorXd result = Q.solve(yvals);
  control_coefs.assign(result.data(), result.data() + result.size());
  ROS_DEBUG_STREAM("coefs: "<<control_coefs[0]<<" "<<control_coefs[1]<<" "<<control_coefs[2]<<" "<<control_coefs[3]);
}

double MPCController::polyeval(double x) {
  double result = control_coefs[0];
  double ax = 1.0;
  for (int i = 1; i<control_coefs.size(); ++i) {
    ax *= x;
    result += ax*control_coefs[i];
  }
  return result;
}

void MPCController::update_robot_pose(double dt)
{
//  ROS_DEBUG_STREAM("update_robot_pose "<<dt<<" v = "<<current_linear_velocity );
  robot_x += current_linear_velocity * dt * cos(robot_theta);
  robot_y += current_linear_velocity * dt * sin(robot_theta);
  robot_theta = angles::normalize_angle(robot_theta + current_angular_velocity * dt);
  robot_time += ros::Duration(dt);
  robot2world.setOrigin(tf::Vector3(robot_x, robot_y, 0));
  robot2world.setRotation(tf::createQuaternionFromYaw(robot_theta));
}

void MPCController::apply_control() {
  cmd_vel += cmd_acc * control_dt;
  cmd_steer_angle += cmd_steer_rate * control_dt;
  cmd_steer_angle = clip<double>(cmd_steer_angle, max_steer_angle);
  //send curvature as command to drives
  std_msgs::Float32 cmd;
  cmd.data = cmd_steer_angle;
  steer_pub.publish(cmd);
  //send velocity as command to drives
  cmd.data = cmd_vel;
  vel_pub.publish(cmd);
  ROS_DEBUG_STREAM("cmd v = "<<cmd_vel<<" angle = "<<cmd_steer_angle);
}


void MPCController::on_timer(const ros::TimerEvent& event)
{
  apply_control();

  //  ROS_INFO_STREAM("on_timer");
  // calculate robot pose to next cycle
  update_robot_pose((event.current_expected - robot_time).toSec() + control_dt );
  update_trajectory_segment();

  update_control_points();
  convert_control_points();
  calculate_control_coefs();

  double error = control_coefs[0];
  ROS_DEBUG_STREAM("error from coef[0] = "<<error);

  const auto start_solve = ros::WallTime::now();
  mpc.solve(current_linear_velocity, cmd_steer_angle, control_coefs, cmd_steer_rate, cmd_acc, mpc_x, mpc_y);
  double solve_time = (ros::WallTime::now() - start_solve).toSec();
  ROS_DEBUG_STREAM("solve time = "<<solve_time);
  ROS_ERROR_STREAM_COND(solve_time > 0.08, "Solve time too big "<<solve_time);
  publish_trajectory();
  publish_poly();
  //send error for debug proposes
  publish_error(cross_track_error());
//  ROS_DEBUG_STREAM("angular_rate cmd = "<<angular_rate);
  publish_mpc_traj(mpc_x, mpc_y);
}

void MPCController::on_pose(const nav_msgs::OdometryConstPtr& odom)
{
  robot_x = odom->pose.pose.position.x;
  robot_y = odom->pose.pose.position.y;
  robot_theta = 2*atan2(odom->pose.pose.orientation.z,
                        odom->pose.pose.orientation.w);

  world_frame_id = odom->header.frame_id;
  robot_time = odom->header.stamp;

//  current_velocity = odom->twist.twist.linear.x;
//  ROS_DEBUG_STREAM("x = "<<robot_x<<" "<<" y = "<<robot_y<<" "<<robot_theta);

//  ROS_DEBUG_STREAM("truth vel = "<<odom->twist.twist.linear.x);
}

void MPCController::on_odo(const nav_msgs::OdometryConstPtr& odom)
{
  current_linear_velocity = odom->twist.twist.linear.x;
  current_angular_velocity = odom->twist.twist.angular.z;
  if ( std::abs(current_linear_velocity) < 0.01 ) {
    current_curvature = current_angular_velocity/ current_linear_velocity;
    current_angle = atan(current_curvature*wheel_base);
  }
  // else left the same as before
  ROS_DEBUG_STREAM("odom vel = "<<current_linear_velocity<<" w = "<<current_angular_velocity<<" angle = "<<current_angle);
}

void MPCController::publish_error(double error)
{
  std_msgs::Float32 err_msg;
  err_msg.data = error;
  err_pub.publish(err_msg);
}

double MPCController::cross_track_error()
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
  return error;
}

void MPCController::get_segment(std::list<TrajPtr>::iterator&  traj_it, double& len)
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

void MPCController::publish_trajectory()
{
  //prepare pointcloud message
  sensor_msgs::PointCloud msg;
  msg.header.frame_id = world_frame_id;
  msg.header.stamp = robot_time;
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
      if ( it == trajectory.end() )
        it = trajectory.begin();
    }
  }
//  ROS_DEBUG_STREAM("publish trajectory");
  traj_pub.publish(msg);
}

void MPCController::publish_poly() {
  //prepare pointcloud message
  sensor_msgs::PointCloud msg;
  msg.header.frame_id = world_frame_id;
  msg.header.stamp = robot_time;
  static int seq(0);
  msg.header.seq = seq++;
  double xrange = control_points_dl * control_points_num * 1.5;
  int trajectory_points_quantity = xrange / traj_dl;
  msg.points.reserve( trajectory_points_quantity );

  for(int i = 0; i<trajectory_points_quantity; ++i) {
    double x = i*traj_dl;
    tf::Vector3 point = robot2world(tf::Vector3(x, polyeval(x), 0));
    add_point(msg, point);
  }
  poly_pub.publish(msg);
}

void MPCController::publish_mpc_traj(std::vector<double>& x, std::vector<double>& y) {
  if (x.empty())
    return;
  sensor_msgs::PointCloud msg;
  msg.header.frame_id = world_frame_id;
  msg.header.stamp = robot_time;
  static int seq(0);
  msg.header.seq = seq++;
  msg.points.reserve( x.size() );
  for( int i = 0; i < x.size(); ++i) {
    add_point(msg, robot2world(tf::Vector3(x[i], y[i], 0)));
  }
  mpc_traj_pub.publish(msg);
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
MPCController::MPCController(const std::string& ns):
    nh("~/" + ns),
    radius( nh.param("radius", 10.0) ),
    cy( nh.param("cy", 2*radius) ), //default circle is in center (0,radius)
    wheel_base( nh.param("wheel_base", 1.88) ),
    max_steer_angle( nh.param("max_steer_angle", 0.3)),
    max_steer_rate( nh.param("max_steer_rate", 0.3) ),
    max_velocity( nh.param("max_velocity", 5.0) ),
    max_acc( nh.param("max_acc", 0.8) ),
    control_dt(nh.param("timer_period", 0.1)),
    traj_dl( nh.param("traj_dl", 0.2) ),
    traj_length( nh.param("traj_length", 5.0) ),
    pose_sub(nh.subscribe("ground_truth", 1, &MPCController::on_pose, this)),
    timer( nh.createTimer( ros::Duration(nh.param("timer_period", 0.1)), &MPCController::on_timer, this ) ),
    err_pub( nh.advertise<std_msgs::Float32>("error", 10) ),
    steer_pub( nh.advertise<std_msgs::Float32>("/steering", 1) ),
    vel_pub( nh.advertise<std_msgs::Float32>("/velocity", 1) ),
    odo_sub( nh.subscribe("odom", 1, &MPCController::on_odo, this)),
    traj_pub( nh.advertise<sensor_msgs::PointCloud>("trajectory", 1) ),
    poly_pub( nh.advertise<sensor_msgs::PointCloud>("poly", 1) ),
    mpc_traj_pub( nh.advertise<sensor_msgs::PointCloud>("mpc_traj", 1) ),
    mpc_steps( nh.param("mpc_steps", 4) ),
    mpc_dt( nh.param("mpc_dt", 0.5) ),
    mpc(mpc_steps, mpc_dt, max_velocity, max_acc, max_steer_angle, max_steer_rate, wheel_base,
        nh.param("kcte", 1.0),
        nh.param("kepsi", 1.0),
        nh.param("kev", 1.0),
        nh.param("ksteer_cost", 1.0))
{
  //counter clock
  trajectory.emplace_back( std::make_shared<trajectory::CircularSegment>( 1.0 / radius,    0,       0,    1.0,   0,   M_PI/2*radius) );
  trajectory.emplace_back( std::make_shared<trajectory::LinearSegment>  (        radius, radius, 0.0,   1.0,  cy - radius) );
  trajectory.emplace_back( std::make_shared<trajectory::CircularSegment>( 1.0 / radius,   radius,   cy,   0.0,   1.0, M_PI/2*radius ) );
  trajectory.emplace_back( std::make_shared<trajectory::CircularSegment>( 1.0 / radius,   0, radius + cy,   -1.0, 0.0, M_PI/2*radius ) );
  trajectory.emplace_back( std::make_shared<trajectory::LinearSegment>  (         -radius, cy,   0.0,   -1.0, cy - radius) );
  trajectory.emplace_back( std::make_shared<trajectory::CircularSegment>( 1.0/ radius,   -radius, radius, 0.0,  -1.0,  M_PI/2*radius) );

  //clock wise track
//  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0 / radius,    0,       0,    1.0,   0,   M_PI/2*radius) );
//  trajectory.emplace_back( std::make_shared<trajectory::LinearTrajectory>  (        radius, -radius, 0.0,   -1.0,  cy - radius) );
//  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0 / radius,   radius,   -cy,   0.0,   -1.0, M_PI/2*radius ) );
//  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0 / radius,   0, -radius - cy,   -1.0,  0.0, M_PI/2*radius ) );
//  trajectory.emplace_back( std::make_shared<trajectory::LinearTrajectory>  (       -radius, -cy,   0.0,   1.0, cy - radius) );
//  trajectory.emplace_back( std::make_shared<trajectory::CircularTrajectory>( -1.0/ radius,   -radius, -radius, 0.0,  1.0,  M_PI/2*radius) );


  current_segment = trajectory.begin();
}


MPCController::~MPCController()
{
  // TODO Auto-generated destructor stub
}

} /* namespace simple_controller */
