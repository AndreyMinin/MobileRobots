
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <algorithm>

ros::Publisher throttle_pub;

double cmd_velocity = 0;
double current_velocity = 0;
ros::Time last_timer_time;

void on_velocity(const std_msgs::Float32& msg) {
  cmd_velocity = msg.data;
  ROS_INFO_STREAM("cmd velocity " << current_velocity << " err = " << cmd_velocity - current_velocity);
}

void on_odo(const nav_msgs::Odometry& odom)
{
  current_velocity = odom.twist.twist.linear.x;
  ROS_INFO_STREAM("current velocity " << current_velocity << " err = " << cmd_velocity - current_velocity);
}


void on_timer(const ros::TimerEvent& event) {
  auto t = ros::Time::now();
  auto dt = (t - last_timer_time).toSec();
  last_timer_time = t;
  std_msgs::Float32 throttle_cmd;
  
  throttle_cmd.data = -20.0;
  throttle_pub.publish(throttle_cmd);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "velocity_controller");
  ros::NodeHandle nh("~");
  throttle_pub = nh.advertise<std_msgs::Float32>("throttle", 1);
  auto odo_sub = nh.subscribe("odom", 1, on_odo);
  auto cmd_sub = nh.subscribe("velocity", 1, on_velocity);
  if (nh.param("/use_sim_time", false)) {
    while(ros::ok()) {
      ros::spinOnce();
    
      last_timer_time = ros::Time::now();
      if (!last_timer_time.isZero()) {
        break;
      } 
    }
  }
  auto timer = nh.createTimer(ros::Duration(0.1), on_timer);
  last_timer_time = ros::Time::now();
  ros::spin();
  return 0;
}


