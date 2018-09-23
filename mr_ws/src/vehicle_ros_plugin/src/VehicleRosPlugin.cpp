/*
 * VehicleRosPlugin.cpp
 *
 *  Created on: 4 апр. 2017 г.
 *      Author: aminin
 */

#include <algorithm>
#include <sdf/sdf.hh>
#include <ros/ros.h>
#include "VehicleRosPlugin.h"
#include <functional>
#include <boost/make_shared.hpp>

const double EPS = 0.00001;

namespace gazebo
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

VehicleRosPlugin::VehicleRosPlugin()
{
  ROS_INFO_STREAM("VehicleRosPlugin created");

}

VehicleRosPlugin::~VehicleRosPlugin()
{
  ROS_INFO_STREAM("VehicleRosPlugin destroyed");
}


void VehicleRosPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->parent = _parent;
  gazebo_ros = boost::make_shared<GazeboRos>(_parent, _sdf, "Vehicle");
  //Make sure the ROS node has already been initialized
  gazebo_ros->isInitialized();

  gazebo_ros->getParameter<std::string>( command_topic, "command_topic", "cmd_vel" );
  gazebo_ros->getParameter<std::string>( odo_topic, "odo_topic", "odom");
  std::string odom_frame;
  gazebo_ros->getParameter<std::string>( odom_frame, "odom_frame", "odom");
  std::string base_frame;
  gazebo_ros->getParameter<std::string>( base_frame, "base_Frame", "base_link");
  gazebo_ros->getParameter<std::string>( steering_angle_topic, "steering_angle_topic", "steering_angle");
  gazebo_ros->getParameter<std::string>( velocity_topic, "velocity_topic", "velocity");
  gazebo_ros->getParameterBoolean(this->pulishJointState, "publishJointState", false );

  gazebo_ros->getParameter<double>( wheel_separation, "wheel_separation", 2.0 );
  gazebo_ros->getParameter<double>( wheel_diameter, "wheel_diameter", 0.5);
  wheel_radius = wheel_diameter/ 2.0;
  gazebo_ros->getParameter<double>( axes_separation, "axes_separation", 3.0);
  gazebo_ros->getParameter<double>( vehicle_accel, "vehicle_accel", 0.0);
  gazebo_ros->getParameter<double>( wheel_torque, "wheel_torque", 5.0);
  gazebo_ros->getParameter<double>( curvature_rate, "curvature_rate", 0.0);
  gazebo_ros->getParameter<double>( steering_torque, "steering_torque", 5.0);
  gazebo_ros->getParameter<double>( update_rate, "update_rate", 100.0);
  gazebo_ros->getParameter<double>( min_radius, "min_radius", 5.0);

  std::map<std::string, Wheels> odomOptions;
  odomOptions["front"] = Wheels::FRONT;
  odomOptions["rear"] = Wheels::REAR;
  gazebo_ros->getParameter<Wheels> ( encoder_wheels, "encoder_wheels", odomOptions, REAR );

  odomOptions["all"] = Wheels::ALL;
  gazebo_ros->getParameter<Wheels> ( drive_wheels, "drive_wheels", odomOptions, REAR );

  joints.resize(JOINTS_NUMBER);
  joints[FRONT_LEFT_WHEEL] = gazebo_ros->getJoint( parent, "front_left_wheel_joint", "front_left_wheel_joint" );
  joints[FRONT_RIGHT_WHEEL] = gazebo_ros->getJoint( parent, "front_right_wheel_joint", "front_right_wheel_joint" );
  joints[REAR_LEFT_WHEEL] = gazebo_ros->getJoint( parent, "rear_left_wheel_joint", "rear_left_wheel_joint" );
  joints[REAR_RIGHT_WHEEL] = gazebo_ros->getJoint( parent, "rear_right_wheel_joint", "rear_right_wheel_joint" );
  joints[FRONT_LEFT_STEERING] = gazebo_ros->getJoint( parent, "front_left_steering_joint", "front_left_steering_joint" );
  joints[FRONT_RIGHT_STEERING] = gazebo_ros->getJoint( parent, "front_right_steering_joint", "front_right_steering_joint" );


  update_period = ( update_rate > 0.0 )? 1.0 / update_rate : 0.0;

  Reset();

  alive = true;

  if ( this->pulishJointState )
  {
    joint_state_pub = gazebo_ros->node()->advertise<sensor_msgs::JointState>("joint_states", 1000);
    ROS_INFO_STREAM(gazebo_ros->info() << " : Advertise joint states");
  }


  odo_pub = gazebo_ros->node()->advertise<nav_msgs::Odometry>("odom", 1000);

  ROS_INFO_STREAM(gazebo_ros->info()<<" : subscribe to cmd_vel");

  cmd_vel_sub = gazebo_ros ->node()->subscribe<geometry_msgs::Twist>(command_topic, 1,
                                                                     &VehicleRosPlugin::cmdVelCallback, this);

  steering_angle_sub = gazebo_ros ->node() ->subscribe<std_msgs::Float32>(steering_angle_topic, 1,
                                                                          &VehicleRosPlugin::steeringAngleCallback, this);

  velocity_sub = gazebo_ros ->node() ->subscribe<std_msgs::Float32>(velocity_topic, 1,
                                                                    &VehicleRosPlugin::velocityCallback, this);
  ros_queue_thread = std::thread( std::bind( &VehicleRosPlugin::QueueThread, this));

  update_connection = event::Events::ConnectWorldUpdateBegin( std::bind( &VehicleRosPlugin::UpdateChild, this) );

  odom_msg.pose.covariance[0] = 0.00001;
  odom_msg.pose.covariance[7] = 0.00001;
  odom_msg.pose.covariance[14] = 100000000000000.0;
  odom_msg.pose.covariance[21] = 100000000000000.0;
  odom_msg.pose.covariance[28] = 100000000000000.0;
  odom_msg.pose.covariance[35] = 0.001;
  odom_msg.header.frame_id = odom_frame;
  odom_msg.child_frame_id = base_frame;
}

void VehicleRosPlugin::Reset()
{
#if GAZEBO_MAJOR_VERSION <= 7
  common::Time current_time = parent->GetWorld()->GetSimTime();
#else
  common::Time current_time = parent->GetWorld()->SimTime();
#endif
  pose_encoder.x = 0;
  pose_encoder.y = 0;
  pose_encoder.theta = 0;
  cmd_velocity = 0;
  cmd_curvature = 0;
  vehicle_curvature = 0;
  vehicle_velocity = 0;
  set_velocity = 0;
  set_curvature = 0;
  if ( drive_wheels == Wheels::FRONT ||
       drive_wheels == Wheels::ALL )
  {
#if GAZEBO_MAJOR_VERSION > 2
    joints[FRONT_LEFT_WHEEL]->SetParam( "fmax", 0, wheel_torque);
    joints[FRONT_RIGHT_WHEEL]->SetParam( "fmax", 0, wheel_torque);
#else
    joints[FRONT_LEFT_WHEEL]->SetMaxForce(  0, wheel_torque);
    joints[FRONT_RIGHT_WHEEL]->SetMaxForce( 0, wheel_torque);
#endif
  }
  if ( drive_wheels == Wheels::REAR ||
       drive_wheels == Wheels::ALL )
  {
#if GAZEBO_MAJOR_VERSION > 2
    joints[REAR_LEFT_WHEEL]->SetParam( "fmax", 0, wheel_torque);
    joints[REAR_RIGHT_WHEEL]->SetParam( "fmax", 0, wheel_torque);
#else
    joints[REAR_LEFT_WHEEL]->SetMaxForce(  0, wheel_torque);
    joints[REAR_RIGHT_WHEEL]->SetMaxForce(  0, wheel_torque);
#endif
  }
#if GAZEBO_MAJOR_VERSION > 2
  joints[FRONT_RIGHT_STEERING]->SetParam( "fmax", 0, steering_torque);
  joints[FRONT_LEFT_STEERING]->SetParam( "fmax", 0, steering_torque);
#else
  joints[FRONT_RIGHT_STEERING]->SetMaxForce(  0, steering_torque);
  joints[FRONT_LEFT_STEERING]->SetMaxForce(  0, steering_torque);
#endif
}

void VehicleRosPlugin::calculateJoints(double curvature, double velocity)
{
  calculateFrontAngles(curvature, velocity);
  if (drive_wheels == Wheels::FRONT || drive_wheels == Wheels::ALL)
  {
    calculateFrontJoints(curvature, velocity);
  }
  if (drive_wheels == Wheels::REAR || drive_wheels == Wheels::ALL)
  {
    calculateRearJoints(curvature, velocity);
  }
}

void VehicleRosPlugin::UpdateChild()
{
  if ( encoder_wheels == Wheels::REAR )
    UpdateRearOdometryEncoder();
  else
    UpdateFrontOdometryEncoder();
#if GAZEBO_MAJOR_VERSION <= 7
  common::Time current_time = parent->GetWorld()->GetSimTime();
#else
  common::Time current_time = parent->GetWorld()->SimTime();
#endif
  double seconds_since_last_udate = (current_time - last_update_time ).Double();

  if ( seconds_since_last_udate > update_period )
  {
//    ROS_INFO_STREAM("seconds_since_last_udate = "<<seconds_since_last_udate);
    publishOdometry();
    if ( this->pulishJointState )
      publishWheelJointStates();

    double curvature, velocity;
    getVehicleCmd(curvature, velocity);
//    ROS_INFO_STREAM("cmd vel = "<<velocity<<" curv = "<<curvature);
    limitCurvatureCmd(curvature, seconds_since_last_udate);
    limitVelocityCmd(velocity, seconds_since_last_udate);
    calculateJoints(curvature, velocity);
    set_velocity = velocity;
    set_curvature = curvature;
//    ROS_INFO_STREAM("set vel = "<<set_velocity<<" curv = "<<set_curvature);
    setJoints();
    last_update_time += common::Time( update_period);
  }
}

void VehicleRosPlugin::setJoints()
{
#if GAZEBO_MAJOR_VERSION > 2
  joints[FRONT_LEFT_STEERING]->SetPosition(0, cmd_left_angle);
  joints[FRONT_RIGHT_STEERING]->SetPosition(0, cmd_right_angle);
#else
  joints[FRONT_LEFT_STEERING]->SetPosition(0, cmd_left_angle);
  joints[FRONT_RIGHT_STEERING]->SetPosition(0, cmd_right_angle);
#endif
//  ROS_INFO_STREAM("set angles "<<cmd_left_angle<<" "<<cmd_right_angle);
  if ( drive_wheels == Wheels::FRONT || drive_wheels == Wheels::ALL )
  {
#if GAZEBO_MAJOR_VERSION > 2
    joints[FRONT_LEFT_WHEEL]->SetParam("vel",0, cmd_rate[FRONT_LEFT_WHEEL]);
    joints[FRONT_RIGHT_WHEEL] ->SetParam("vel", 0, cmd_rate[FRONT_RIGHT_WHEEL]);
#else
    joints[FRONT_LEFT_WHEEL]->SetVelocity(0, cmd_rate[FRONT_LEFT_WHEEL]);
    joints[FRONT_RIGHT_WHEEL]->SetVelocity(0, cmd_rate[FRONT_RIGHT_WHEEL]);
#endif
//    ROS_INFO_STREAM("set front wheel rate "<<cmd_rate[FRONT_LEFT_WHEEL]<<" "<<cmd_rate[FRONT_RIGHT_WHEEL]);
  }
  if ( drive_wheels == Wheels::REAR || drive_wheels == Wheels::ALL )
  {
#if GAZEBO_MAJOR_VERSION > 2
    joints[REAR_LEFT_WHEEL]->SetParam("vel",0, cmd_rate[REAR_LEFT_WHEEL]);
    joints[REAR_RIGHT_WHEEL] ->SetParam("vel", 0, cmd_rate[REAR_RIGHT_WHEEL]);
#else
    joints[REAR_LEFT_WHEEL]->SetVelocity(0, cmd_rate[REAR_LEFT_WHEEL]);
    joints[REAR_RIGHT_WHEEL]->SetVelocity( 0, cmd_rate[REAR_RIGHT_WHEEL]);
#endif
//    ROS_INFO_STREAM("set rear wheel rate "<<cmd_rate[REAR_LEFT_WHEEL]<<" "<<cmd_rate[REAR_RIGHT_WHEEL]);
  }
}

void VehicleRosPlugin::FiniChild()
{
  alive = false;
  ros_queue.clear();
  ros_queue.disable();
  gazebo_ros->node()->shutdown();
  ros_queue_thread.join();
}

void VehicleRosPlugin::publishOdometry()
{
  odom_msg.pose.pose.position.x = pose_encoder.x;
  odom_msg.pose.pose.position.y = pose_encoder.y;
  odom_msg.pose.pose.position.z = 0;

  odom_msg.pose.pose.orientation.x = 0;
  odom_msg.pose.pose.orientation.y = 0;
  odom_msg.pose.pose.orientation.z = sin ( pose_encoder.theta / 2.0 );
  odom_msg.pose.pose.orientation.w = cos ( pose_encoder.theta / 2.0 );

  odom_msg.twist.twist.angular.z = vehicle_velocity * vehicle_curvature;
  odom_msg.twist.twist.linear.x = vehicle_velocity;
  odom_msg.twist.twist.linear.y = 0;

  odom_msg.header.stamp = ros::Time::now();
  odo_pub.publish(odom_msg);
}

void VehicleRosPlugin::calculateFrontAngles(double curvature, double velocity)
{
  if ( fabs ( curvature) < EPS )
  {
    cmd_left_angle = 0.0;
    cmd_right_angle = 0.0;
    return;
  }
  //else
  double R = 1.0 / curvature;

  if ( R > 0 && R < min_radius )
    R = min_radius;
  else if ( R < 0 && R > -min_radius )
    R = -min_radius;

  double left_angle =  atan2(wheel_separation/2.0 - R,  axes_separation);
  double right_angle = atan2(-wheel_separation/2.0 - R, axes_separation);

  if ( R > 0 )
  {
    cmd_left_angle = M_PI/2 + left_angle;
    cmd_right_angle = M_PI/2 + right_angle;
  }
  else
  {
    cmd_left_angle = -M_PI/2 + left_angle;
    cmd_right_angle = -M_PI/2 + right_angle;
  }

}


void VehicleRosPlugin::calculateRearJoints(double curvature, double velocity)
{
  cmd_rate[REAR_LEFT_WHEEL]  = velocity * (1.0  - wheel_separation/2.0 * curvature)/ wheel_radius;
  cmd_rate[REAR_RIGHT_WHEEL]  = velocity * (1.0 + wheel_separation/2.0 * curvature) / wheel_radius;
}

void VehicleRosPlugin::calculateFrontJoints(double curvature, double velocity)
{
  cmd_rate[FRONT_LEFT_WHEEL]  = velocity / cos( cmd_left_angle )/ wheel_radius;
  cmd_rate[FRONT_RIGHT_WHEEL]  = velocity / cos( cmd_right_angle )/ wheel_radius;
}

void VehicleRosPlugin::limitVelocityCmd( double& velocity, double dt)
{
  if ( vehicle_accel > EPS && fabs(set_velocity - velocity ) > EPS )
  {
    if ( velocity > set_velocity )
      velocity = set_velocity + std::min(velocity - set_velocity, vehicle_accel*dt);
    else
      velocity = set_velocity + std::max(velocity - set_velocity, -vehicle_accel*dt);
  }
}

void VehicleRosPlugin::limitCurvatureCmd(double& curvature, double dt)
{
  if ( curvature_rate > EPS && (fabs(set_curvature - curvature ) > EPS) )
  {
    if ( curvature > set_curvature )
      curvature = set_curvature + std::min(curvature - set_curvature, curvature_rate*dt);
    else
      curvature = set_curvature + std::max(curvature - set_curvature, -curvature_rate*dt);
  }
}

void VehicleRosPlugin::getVehicleCmd(double& c, double& v)
{
  std::lock_guard<std::mutex> scoped_lock( lock );
  c = cmd_curvature;
  v = cmd_velocity;
}

void VehicleRosPlugin::publishWheelJointStates()
{
  ros::Time current_time = ros::Time::now();

  joint_state_msg.header.stamp = current_time;
  joint_state_msg.name.resize( joints.size() );
  joint_state_msg.position.resize( joints.size() );
  joint_state_msg.velocity.resize( joints.size() );

  for( int i = 0; i< JOINTS_NUMBER; ++i )
  {
    physics::JointPtr& joint = joints[i];
#if GAZEBO_MAJOR_VERSION <= 7
    joint_state_msg.position[i] = joint->GetAngle(0).Radian();
#else
    joint_state_msg.position[i] = joint->Position( 0 );
#endif
    joint_state_msg.velocity[i] = joint->GetVelocity(0);
    joint_state_msg.name[i] = joint->GetName();
  }
  joint_state_pub. publish( joint_state_msg );
}

void VehicleRosPlugin::UpdateFrontOdometryEncoder()
{
#if GAZEBO_MAJOR_VERSION <= 7
  double al = joints[FRONT_LEFT_STEERING] -> GetAngle( 0 ).Radian();
  double ar = joints[FRONT_RIGHT_STEERING] -> GetAngle( 0 ).Radian();
  common::Time current_time = parent->GetWorld()->GetSimTime();
#else
  double al = joints[FRONT_LEFT_STEERING] -> Position( 0 );
  double ar = joints[FRONT_RIGHT_STEERING] -> Position( 0 );
  common::Time current_time = parent->GetWorld()->SimTime();
#endif
  double vl = wheel_radius*joints[FRONT_LEFT_WHEEL]->GetVelocity( 0 );
  double vr = wheel_radius*joints[FRONT_RIGHT_WHEEL]->GetVelocity( 0 );


  double seconds_since_last_update = (current_time - last_odom_update).Double();
  last_odom_update = current_time;

  vl *= cos(al);
  vr *= cos(ar);
  vehicle_velocity = (vl  + vr  )/2.0;
  double w = (vr - vl ) /(2.0 * wheel_separation);
  vehicle_curvature = (fabs(vehicle_velocity) > EPS)? (w / vehicle_velocity) : 0.0;
}


void VehicleRosPlugin::UpdateRearOdometryEncoder()
{
  double vl = joints[REAR_LEFT_WHEEL]->GetVelocity( 0 );
  double vr = joints[REAR_RIGHT_WHEEL]->GetVelocity( 0 );
#if GAZEBO_MAJOR_VERSION <= 7
  common::Time current_time = parent->GetWorld()->GetSimTime();
#else
  common::Time current_time = parent->GetWorld()->SimTime();
#endif
  double seconds_since_last_update = (current_time - last_odom_update).Double();
  last_odom_update = current_time;

  double b = wheel_separation;

  double sl = vl * ( wheel_radius ) * seconds_since_last_update;
  double sr = vr * ( wheel_radius ) * seconds_since_last_update;
  double ssum = sl + sr;
  double sdiff = sr - sl;

  double dx = ( ssum ) /2.0 * cos ( pose_encoder.theta + ( sdiff ) / ( 2.0*b ) );
  double dy = ( ssum ) /2.0 * sin ( pose_encoder.theta + ( sdiff ) / ( 2.0*b ) );
  double dfi = sdiff / b;

  pose_encoder.x += dx;
  pose_encoder.y += dy;
  pose_encoder.theta += dfi;

  double w = dfi / seconds_since_last_update;
  double vx = dx / seconds_since_last_update;
  double vy = dy / seconds_since_last_update;

  vehicle_velocity = ssum/2.0/ seconds_since_last_update;
  vehicle_curvature = (fabs(vehicle_velocity) > EPS)? (w / vehicle_velocity) : 0.0;
}

void VehicleRosPlugin::QueueThread()
{
  static const double timeout = 0.01;
  while(alive && gazebo_ros->node()->ok() )
  {
    ros_queue.callAvailable( ros::WallDuration( timeout ) );
  }
}

void VehicleRosPlugin::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{

  std::lock_guard<std::mutex> scoped_lock( lock );
  cmd_velocity = cmd_msg->linear.x;
  cmd_curvature = (fabs(cmd_velocity) > EPS )? cmd_msg->angular.z / cmd_velocity : 0.0;
}

void VehicleRosPlugin::steeringAngleCallback(const std_msgs::Float32ConstPtr& angle_cmd)
{
  std::lock_guard<std::mutex> scoped_lock( lock );
  cmd_curvature = clip<double>( angle_cmd->data, 1.0 / min_radius);

}

void VehicleRosPlugin::velocityCallback(const std_msgs::Float32ConstPtr& vel_cmd)
{
  std::lock_guard<std::mutex> scoped_lock( lock );
  cmd_velocity = vel_cmd->data;
}

GZ_REGISTER_MODEL_PLUGIN( VehicleRosPlugin )

} /* namespace vehicle_ros_plugin */
