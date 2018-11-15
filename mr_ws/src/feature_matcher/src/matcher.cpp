#include "matcher.h"

void Matcher::on_laser_scan(const sensor_msgs::LaserScan& scan)
{
  ROS_INFO_STREAM("on laser scan");
  detect_features(scan);
  find_feature_pairs();
  find_transform();
  publish_transform(scan.header);
  publish_features(scan.header);
  update_base_features();
}

void Matcher::publish_features(const std_msgs::Header& header)
{
  visualization_msgs::Marker marker;
  marker.header = header;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.points.resize(new_features.size());
  for (std::size_t i = 0; i < marker.points.size(); ++i) {
    marker.points[i].x = new_features[i].x();
    marker.points[i].y = new_features[i].y();
    marker.points[i].z = 0;
  }
  marker.id = 100;
  marker.ns = "features";
}

void Matcher::detect_features(const sensor_msgs::LaserScan& scan)
{
}

void Matcher::find_feature_pairs() {

}

void Matcher::find_transform()
{
}

void Matcher::publish_transform(const std_msgs::Header& header)
{
  nav_msgs::Odometry odo;
  odo.header.stamp = header.stamp;

  const auto& matrix = transform.matrix();

  double yaw = atan2(matrix(1, 0), matrix(0, 0));
  odo.pose.pose.position.x = transform.translation().x();
  odo.pose.pose.position.y = transform.translation().y();

  odo_pub.publish(odo);
}

void Matcher::update_base_features()
{
  feature_pair_indices.clear();
  base_features = new_features;
}
