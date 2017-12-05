#include "slam.h"
#include <angles/angles.h>
#include <strstream>

void Slam::on_odo(const nav_msgs::Odometry& odom)
{
  ROS_INFO_STREAM("on odo");
  v = odom.twist.twist.linear.x;
  w = odom.twist.twist.angular.z;
}

void Slam::on_scan(const sensor_msgs::LaserScan& scan)
{
  ROS_INFO_STREAM("on scan");

  predict((scan.header.stamp - last_time).toSec());
  last_time = scan.header.stamp;
}

void Slam::on_timer(const ros::TimerEvent& event)
{
}

void Slam::publish_results(const std::string& frame, ros::Time& time)
{
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.frame_id = frame;
  pose.header.stamp = time;
  static int seq = 0;
  ++seq;
  pose.header.seq = seq;
  pose.pose.covariance.assign(0);
  pose.pose.covariance[0] = P(0,0); pose.pose.covariance[1] = P(0,1); pose.pose.covariance[2] = 0;
  pose.pose.covariance[3] = P(0,0); pose.pose.covariance[1] = P(0,1); pose.pose.covariance[2] = P(0,2);
  pose.pose.pose.position.x = X(0);
  pose.pose.pose.position.y = X(1);
  pose.pose.pose.position.z = 0;
  pose.pose.pose.orientation.x = 0;
  pose.pose.pose.orientation.y = 0;
  pose.pose.pose.orientation.w = cos(X(2)/2);
  pose.pose.pose.orientation.z = sin(X(2)/2);


  pose_pub.publish(pose);


}

void Slam::predict(double dt)
{
  X(0) += v * cos(X(2)) * dt;
  X(1) += v * sin(X(2)) * dt;
  X(2) += w * dt;
  X(2) = angles::normalize_angle(X(2));

  A = Eigen::Matrix2d::Identity(X.size(),X.size());
  A(0,0) = 1.0; A(0,1) = 0; A(0,2) = -v * sin(X(2)) * dt;
  A(1,0) = 0.0; A(1,1) = 1.0; A(1,2) = v * cos(X(2)) * dt;
  A(2,0) = 0.0; A(2,1) = 0.0; A(2,2) = 1.0;

  P = A * P * A.transpose() + R;
}

Slam::Slam():
    nh("~"),
    odo_sub(nh.subscribe("odom", 1, &Slam::on_odo, this)),
    scan_sub(nh.subscribe("/scan", 1, &Slam::on_scan, this)),
    pose_pub(nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("slam_pose", 1)),
    timer(nh.createTimer(ros::Duration(0.1), &Slam::on_timer, this)),
    X(3 + 2*NUMBER_LANDMARKS, 0),
    A(Eigen::Matrix2d::Identity(X.size(),X.size())),
    P(X.size(), X.size()),
    R(X.size(), X.size())
{
  std::string landmark("landmark");
  for (int i = 0; i<NUMBER_LANDMARKS; ++i) {
    std::strstream stream;
    stream<<landmark<<i;
    landmark_pub[i] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(stream.str(), 1);
  }
  R = Eigen::Matrix2d::Zero(P.rows(), P.cols());
  R(0,0) = 0.1;
  R(1,1) = 0.1;
  R(2,2) = 0.01;

}
