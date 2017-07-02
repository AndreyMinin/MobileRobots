/*
 * laser_map.cpp
 *
 *  Created on: 25 июн. 2017 г.
 *      Author: aminin
 */

#include "laser_map.h"


namespace laser2d_map
{

const std::string LAYER_NAME("laser_occupancy");

bool LaserMap::check_scan_number(const sensor_msgs::LaserScanConstPtr& scan)
{
  if (scan->header.seq < last_scan_number + miss_scan_number)
     return false;
  return true;
}

bool LaserMap::check_scan_latency(const sensor_msgs::LaserScanConstPtr& scan)
{
  ros::Time current_time( ros::Time::now() );
  double scan_latency = (scan->header.stamp - current_time).toSec();
  if ( scan_latency  > max_latency )
  {
    ROS_WARN_STREAM("scan latency exceeded "<<scan_latency);
    return false;
  }
  return true;
}

bool LaserMap::update_transform(const sensor_msgs::LaserScanConstPtr& scan)
{
  try
  {
    if ( !tf.waitForTransform(map_frame, scan->header.frame_id, scan->header.stamp, ros::Duration(0.1) ) )
    {
      ROS_WARN_STREAM("wait transform time exceeded");
      return false;
    }

    tf.lookupTransform(map_frame, scan->header.frame_id, scan->header.stamp, scan_transform );
  }
  catch (tf::TransformException& e) {
    ROS_WARN_STREAM("update scan transform exception "<<e.what() );
    return false;
  }
  return true;
}

void LaserMap::draw_line(grid_map::Index& index0, grid_map::Index& index1,  float color )
{
//  ROS_DEBUG_STREAM("draw line from ("<<index0(0)<<" "<<index0(1)<<") to ("<<index1(0)<<" "<<index1(1)<<")");
  for( grid_map::LineIterator it(map, index0, index1 );
                !it.isPastEnd();  ++it)
  {
    const grid_map::Index index(*it);
    map_layer(index(0), index(1)) = color;
  }
}

void LaserMap::draw_obstacles(const std::list<grid_map::Index>& indexes)
{
//  ROS_DEBUG_STREAM("draw obstacles "<<indexes.size());
  for( auto index : indexes)
  {
//    ROS_DEBUG_STREAM( index(0)<<" "<< index(1));
    map_layer( index(0), index(1) ) = obstacle_value;
  }
}

void LaserMap::on_laser_scan(const sensor_msgs::LaserScanConstPtr& scan)
{
//  ROS_DEBUG_STREAM("on scan");

  if ( ! check_scan_number(scan) )
    return;
  if ( ! check_scan_latency(scan) )
    return;

  last_scan_number = scan->header.seq;

  if ( ! update_transform(scan) )
    return;


  tf::Vector3& laser_pose = scan_transform.getOrigin();

  grid_map::Position laser_position(laser_pose.x(), laser_pose.y());
//  ROS_INFO_STREAM("scan pose "<<laser_pose.x()<<" "<<laser_pose.y());


  //move map to the center of current laser position
  map.move( laser_position );
  map.convertToDefaultStartIndex();

  grid_map::Index laser_index;
  if ( !map.getIndex(laser_position, laser_index) )
  {
    ROS_ERROR_STREAM("laser pose is outside the map");
  }

  max_map_range = std::min<double>(scan->range_max, max_map_range);

  //select rays belonging to obstacles
  std::list<grid_map::Index> obstacle_indexes;
  for( int i = 0; i<scan->ranges.size(); i += (1 + miss_ray) )
  {
    bool obstacle = true;
    auto range = scan->ranges[i];
    if ( range < min_range )
      return;
    //TODO perhaps only for gazebo
    if ( std::isinf(range) ||
        (range > max_map_range))
    {
      obstacle = false;
      range = scan->range_max;
    }

    double ray_angle = scan->angle_min + i * scan->angle_increment;
    //point in laser coordinates
    tf::Vector3 laser_point(range * cos( ray_angle ),
                            range * sin( ray_angle ),
                            0);
    //point in world coordinates
    tf::Vector3 point = scan_transform(laser_point);
    grid_map::Index point_index;
    //point is to be in map
    if ( ! map.getIndex( grid_map::Position(point.x(), point.y()), point_index ) )
    {
      ROS_ERROR_STREAM("point pose is outside the map");
    }

    if ( obstacle )
    {
//      - add its index to list
        obstacle_indexes.push_back( point_index );
    }
    //draw line of clear cells on map
    draw_line( laser_index, point_index, free_value);
  }

  draw_obstacles(obstacle_indexes);


  grid_map::GridMapRosConverter::toOccupancyGrid(map, LAYER_NAME, 0, 100, map_msg);

  map_msg.header.stamp = scan->header.stamp;
//  ROS_DEBUG_STREAM("publish map ");
  map_pub.publish(map_msg);
}

void LaserMap::init_map()
{
  map.setFrameId(map_frame);
  map.setGeometry(grid_map::Length(x_width, y_width), resolution);
}

LaserMap::LaserMap(ros::NodeHandle& nh):
        map_frame( nh.param<std::string>("map_frame", "map" ) ),
        resolution( nh.param("resolution", 0.1) ),
        x_width( nh.param("x_width", 40.0 ) ),
        y_width( nh.param("y_width", 40.0 ) ),
        last_scan_time( ros::Time::now() ),
        max_latency( nh.param("max_latency", max_latency) ),
        miss_scan_number( nh.param("miss_scan_number", 0) ),
        last_scan_number( -miss_scan_number ), //to allow first scan
        miss_ray( nh.param("miss_ray", 0) ),
        max_map_range(std::min(x_width, y_width) / 2.0 - resolution),
        min_range( nh.param("min_range", 0.0) ),
        map( { LAYER_NAME } ),
        map_layer( map[LAYER_NAME] ),
        scan_sub( nh.subscribe("/laser_scan", 1, &LaserMap::on_laser_scan, this ) ),
        map_pub( nh.advertise<nav_msgs::OccupancyGrid>("laser_map", 10) ),
        free_value( nh.param("free_value", 0) ),
        obstacle_value( nh.param("obstacle_value", 100) )
{
  init_map();

}



LaserMap::~LaserMap()
{
  // TODO Auto-generated destructor stub
}

} /* namespace laser2d_map */
