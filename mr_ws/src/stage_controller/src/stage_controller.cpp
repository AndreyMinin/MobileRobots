/*
 * stage_controller.cpp
 *
 *  Created on: Oct 3, 2018
 *      Author: andreyminin
 */

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>
#include <algorithm>

double car_length = 1.5;

double steering = 0;
double cmd_steering = 0.0;
double max_steering_rate = 1.0;
double max_steering = 0.5;
double velocity = 0;
double desired_velocity = 0;
double max_acc = 1.0;
double max_velocity = 10;
ros::Publisher twist_pub;
ros::Time last_timer_time;

void on_steering(const std_msgs::Float32& msg) {
  // ROS_INFO_STREAM_COND(std::abs(msg.data - cmd_steering) > 0.1, " cmd steering = " << msg.data);
  cmd_steering = msg.data;

}

void on_command_velocity(const std_msgs::Float32& msg) {
  // ROS_INFO_STREAM_COND(std::abs(msg.data - desired_velocity) > 0.1, " cmd velocity = " << msg.data);
  desired_velocity = msg.data;
}

double clamp(double cmd, double max_value)
{
  return copysign(std::min(std::abs(cmd), max_value), cmd);
}

double clamp(double value, double cmd, double max_value, double max_rate, double dt) {
  cmd = clamp(cmd, max_value);
  auto diff = (cmd - value);
  diff = clamp(diff, max_rate * dt);
  return value + diff;
}

void on_timer(const ros::TimerEvent& event) {
  auto t = ros::Time::now();
  auto dt = (t - last_timer_time).toSec();
  last_timer_time = t;
  geometry_msgs::Twist cmd;
  velocity = clamp(velocity, desired_velocity, max_velocity, max_acc, dt);
  steering = clamp(steering, cmd_steering, max_steering, max_steering_rate, dt);
  // ROS_INFO_STREAM("v = " << velocity <<" s = " << steering);
  cmd.linear.x = velocity;
  cmd.angular.z = velocity * tan(steering) / car_length;
  twist_pub.publish(cmd);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "stage_controller");
  ros::NodeHandle nh("~");
  car_length = nh.param("length", 1.5);
  ROS_INFO_STREAM("car length " << car_length);
  max_steering = nh.param("max_steering", 0.5);
  max_steering_rate = nh.param("max_steering_rate", 1.0);
  max_acc = nh.param("max_acc", 1.5);
  max_velocity = nh.param("max_velocity", 15);
  twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  auto timer = nh.createTimer(ros::Duration(0.05), on_timer);
  auto steer_sub = nh.subscribe("steering", 1, on_steering);
  auto vel_sub = nh.subscribe("velocity", 1, on_command_velocity);
  last_timer_time = ros::Time::now();
  ros::spin();
  return 0;
}


