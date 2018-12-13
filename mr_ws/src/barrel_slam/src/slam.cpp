#include "slam.h"
#include <angles/angles.h>
#include <sstream>
#include <math.h>

void Slam::on_odo(const nav_msgs::Odometry& odom)
{
  v = odom.twist.twist.linear.x;
  w = odom.twist.twist.angular.z;
}

std::vector<Eigen::Vector2d> Slam::detect_landmarks(const sensor_msgs::LaserScan& scan)
{

}

int Slam::associate_measuriment(const Eigen::Vector2d& landmark_measuriment)
{
  double nearest_distance = 1e10;
  int nearest_index = -1;
  for (std::size_t i = 0; i < NUMBER_LANDMARKS; ++i) {
    double distance = (landmark_measuriment - X.segment(i*2, 2)).norm();
    if (distance < nearest_distance) {
      nearest_index = i;
    }
  }
  // naive association
  const double kAssocThreshold = 5.0;
  if (nearest_index < kAssocThreshold)
  {
    return nearest_index;
  }
  return -1;
}

int Slam::add_landmark(const Eigen::Vector2d& landmark_measuriment)
{
  ++last_found_landmark_index;
  return last_found_landmark_index;
}

void Slam::correct(int index, const Eigen::Vector2d& landmark_measuriment)
{
}

void Slam::on_scan(const sensor_msgs::LaserScan& scan)
{
  predict((scan.header.stamp - last_time).toSec());
  last_time = scan.header.stamp;


}

void Slam::on_timer(const ros::TimerEvent& event)
{
  publish_results("odom", last_time);
}

void fill_pose_msg(geometry_msgs::PoseWithCovariance& pose,
                   double x, double y, double fi,
                   const Eigen::Matrix2d& cov_matr)
{
  pose.covariance.assign(0);
  pose.covariance[0] = cov_matr(0,0); pose.covariance[1] = cov_matr(0,1);
  pose.covariance[6] = cov_matr(1,0); pose.covariance[7] = cov_matr(1,1);
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.w = cos(fi/2);
  pose.pose.orientation.z = sin(fi/2);
}

void Slam::publish_results(const std::string& frame, ros::Time& time)
{
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.frame_id = frame;
  pose.header.stamp = time;
  fill_pose_msg(pose.pose, X(0), X(1), X(2), P.topLeftCorner(2, 2));
  pose_pub.publish(pose);

  const double kPublishCov = 100;
  for (int i = 0; i < NUMBER_LANDMARKS; ++i) {
    if (P(ROBOT_VARS + i * 2, ROBOT_VARS + i * 2) < kPublishCov &&
        P(ROBOT_VARS + i * 2 + 1, ROBOT_VARS + i * 2 + 1) < kPublishCov)
    {
      geometry_msgs::PoseWithCovarianceStamped pose;
      pose.header.frame_id = frame;
      pose.header.stamp = time;
      fill_pose_msg(pose.pose, X(ROBOT_VARS + i * 2), X(ROBOT_VARS + i * 2 + 1), 0,
                    P.block<2, 2>(ROBOT_VARS + i * 2, ROBOT_VARS + i * 2));
      landmark_pub[i].publish(pose);
    }
  }

}

void Slam::predict(double dt)
{
  X(0) += v * cos(X(2)) * dt;
  X(1) += v * sin(X(2)) * dt;
  X(2) += w * dt;
  X(2) = angles::normalize_angle(X(2));

  A = Eigen::Matrix3d::Identity();
  A(0,0) = 1.0; A(0,1) = 0; A(0,2) = -v * sin(X(2)) * dt;
  A(1,0) = 0.0; A(1,1) = 1.0; A(1,2) = v * cos(X(2)) * dt;
  A(2,0) = 0.0; A(2,1) = 0.0; A(2,2) = 1.0;

  P.topLeftCorner(ROBOT_VARS, ROBOT_VARS) =
      A * P.topLeftCorner(ROBOT_VARS, ROBOT_VARS) * A.transpose() + D;

  P.topRightCorner(ROBOT_VARS, NUMBER_LANDMARKS * 2) = A * P.topRightCorner(ROBOT_VARS, NUMBER_LANDMARKS * 2);
  P.bottomLeftCorner(NUMBER_LANDMARKS * 2, ROBOT_VARS) = P.topRightCorner(ROBOT_VARS, NUMBER_LANDMARKS * 2).transpose();
}

void Slam::advertize_landmark_publishers()
{
  std::string landmark("landmark");
  for (int i = 0; i < NUMBER_LANDMARKS; ++i)
  {
    std::stringstream stream;
    stream << landmark << i;
    landmark_pub[i] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(stream.str(), 1);
  }
}



Slam::Slam():
    nh("~"),
    odo_sub(nh.subscribe("/odom", 1, &Slam::on_odo, this)),
    scan_sub(nh.subscribe("/scan", 1, &Slam::on_scan, this)),
    pose_pub(nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("slam_pose", 1)),
    timer(nh.createTimer(ros::Duration(0.1), &Slam::on_timer, this)),
    X(3 + 2*NUMBER_LANDMARKS),
    A(Eigen::Matrix3d::Identity()),
    P(Eigen::MatrixXd::Zero(X.size(), X.size()))
{
  X = Eigen::VectorXd::Zero(X.size());
  P.bottomRightCorner(NUMBER_LANDMARKS * 2, NUMBER_LANDMARKS * 2) =
      HUGE_COVARIANCE * Eigen::MatrixXd::Identity(NUMBER_LANDMARKS * 2, NUMBER_LANDMARKS * 2);

  advertize_landmark_publishers();
  R = Eigen::Matrix2d::Zero();
  R(0, 0) = 0.01;
  R(1, 1) = 0.001;

  D = Eigen::Matrix3d::Zero();
  D(0, 0) = 0.01;
  D(1, 1) = 0.01;
  D(2, 2) = 0.001;
}
