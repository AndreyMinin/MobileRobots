/*
 * laser_map.h
 *
 *  Created on: 25 июн. 2017 г.
 *      Author: aminin
 */

#ifndef SRC_LASER_MAP_H_
#define SRC_LASER_MAP_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

namespace laser2d_map
{
/*
 * \brief module for projecting laserscan to map
 * subscribes to laserscan message
 * subscribes to tf
 * projects every scan to grid_map and  publishes as Occupancygrid message
 */
class LaserMap
{
protected:
  std::string map_frame;
  double resolution;   //in meters per cell
  double x_width;  //in meters
  double y_width;   //in meters
  nav_msgs::OccupancyGrid map_msg;

  ros::Time last_scan_time;
  double max_latency;
  //scan filtering parameters
  int miss_scan_number;
  int last_scan_number;
  //only every miss ray will be processed
  int miss_ray;

  double max_map_range;
  double min_range;

  tf::TransformListener tf;
  tf::StampedTransform scan_transform;

  grid_map::GridMap map;
  grid_map::Matrix& map_layer;

  ros::Subscriber scan_sub;
  ros::Publisher map_pub;

  float free_value;
  float obstacle_value;

  bool check_scan_number(const sensor_msgs::LaserScanConstPtr& scan);
  bool check_scan_latency(const sensor_msgs::LaserScanConstPtr& scan);

  bool update_transform(const sensor_msgs::LaserScanConstPtr& scan);

  void on_laser_scan(const sensor_msgs::LaserScanConstPtr& scan);
  void init_map();

  void draw_line(grid_map::Index& index0, grid_map::Index& index1,  float color );

  void draw_obstacles(const std::list<grid_map::Index>& indexes);
public:

  LaserMap(ros::NodeHandle& nh);
  virtual ~LaserMap();
};

} /* namespace laser2d_map */

#endif /* SRC_LASER_MAP_H_ */
