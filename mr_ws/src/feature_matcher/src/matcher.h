#ifndef SRC_FEATURE_MATCHER_SRC_MATCHER_H_
#define SRC_FEATURE_MATCHER_SRC_MATCHER_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <vector>

class Matcher
{
public:
  explicit Matcher(ros::NodeHandle& nh) : nh(nh) {}

private:
  void on_laser_scan(const sensor_msgs::LaserScan& scan);
  void detect_features(const sensor_msgs::LaserScan& scan);
  void find_feature_pairs();
  void find_transform();
  void publish_transform(const std_msgs::Header& header);
  void publish_features(const std_msgs::Header& header);
  void update_base_features();

private:
  ros::NodeHandle nh;
  ros::Subscriber laser_sub = nh.subscribe("/base_scan", 1, &Matcher::on_laser_scan, this);
  ros::Publisher feature_pub = nh.advertise<visualization_msgs::Marker>("features", 1, true);
  ros::Publisher odo_pub = nh.advertise<nav_msgs::Odometry>("odo", 1 , false);
  // features from base scan
  std::vector<Eigen::Vector2d> base_features;
  // features from new scan
  std::vector<Eigen::Vector2d> new_features;

  // indexes for new features for every base feature
  std::vector<std::size_t> feature_pair_indices;

  Eigen::Isometry2d transform = Eigen::Isometry2d::Identity();
};

#endif /* SRC_FEATURE_MATCHER_SRC_MATCHER_H_ */
