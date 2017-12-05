#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>

const int NUMBER_LANDMARKS=12;

class Slam {
private:
  ros::NodeHandle nh;
  ros::Subscriber odo_sub;
  ros::Subscriber scan_sub;
  // публикатор положения робота
  ros::Publisher pose_pub;
  // публикатор положений маяков
  ros::Publisher landmark_pub[NUMBER_LANDMARKS];
  ros::Timer timer;
  void on_odo(const nav_msgs::Odometry& odom);
  void on_scan(const sensor_msgs::LaserScan& scan);
  void on_timer(const ros::TimerEvent& event);
  void publish_results(const std::string& frame, ros::Time& time);
  void predict(double dt);

  double v = 0;
  double w = 0;

  Eigen::Vector2d X;
  Eigen::Matrix2Xd A;
  Eigen::Matrix2Xd P;
  Eigen::Matrix2Xd R;
  Eigen::Matrix2Xd K;

  ros::Time last_time = ros::Time::now();

public:
  Slam();
};
