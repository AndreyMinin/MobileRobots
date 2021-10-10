/*
 * stage_controller.cpp
 *
 *  Created on: Oct 3, 2018
 *      Author: andreyminin
 */

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <algorithm>

ros::Publisher test_pub;
double desired_velocity = 0;
double max_velocity = 10;
double acc = 1.0;
double max_test_time = 5.0;
double acc_time = max_test_time / 2.;
double dcc_time = acc_time;
ros::Time start_test_time;
bool started = false;

void on_odo(const nav_msgs::Odometry& odom)
{
  double current_velocity = odom.twist.twist.linear.x;

}

void on_timer(const ros::TimerEvent& event) {
  const auto t = ros::Time::now();
  if (!started) {
    start_test_time = t;
    started = true;
  }

  auto test_time = (t - start_test_time).toSec();

  std_msgs::Float32 vcmd;
  if (test_time >= max_test_time) {
    desired_velocity = 0;
  } else {
    if (test_time <= acc_time) {
      desired_velocity = std::min(max_velocity, test_time * acc);
    } else {
      if (test_time >= dcc_time) {
        desired_velocity = std::max(0.0, max_velocity - acc * (test_time - dcc_time));
      } else {
        desired_velocity = max_velocity;
      }
    }
  }
  ROS_INFO_STREAM_COND(desired_velocity > 0.01, "test_time = " << test_time << " test vel = " << desired_velocity);
  vcmd.data = desired_velocity;
  test_pub.publish(vcmd);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "velocity_test");
  ros::NodeHandle nh("~");
  max_test_time = nh.param("test_time", 10.0);
  acc = nh.param("acc", 1.0);
  max_velocity = nh.param("max_velocity", 7.0);

  if (max_velocity / acc > max_test_time / 2.0) {
    acc_time = max_test_time / 2.0;
    dcc_time = max_test_time / 2.0;
    max_velocity = acc * max_test_time / 2.0;
    ROS_WARN_STREAM("Not enough time to reach max velocity " << max_velocity);
  } else {
    acc_time = max_velocity / acc;
    dcc_time = max_test_time - acc_time;
  }
  
  auto odo_sub = nh.subscribe("odom", 1, on_odo);
  test_pub = nh.advertise<std_msgs::Float32>("velocity", 1);

  if (nh.param("/use_sim_time", false)) {
    while(ros::ok()) {
      ros::spinOnce();
    
      start_test_time = ros::Time::now();
      if (!start_test_time.isZero()) {
        break;
      } 
    }
  }
  auto timer = nh.createTimer(ros::Duration(0.1), on_timer);
  start_test_time = ros::Time::now();
  ros::spin();
  return 0;
}


