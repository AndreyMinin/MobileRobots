/*
 * point_trajectory.cpp
 *
 *  Created on: 2 июл. 2017 г.
 *      Author: aminin
 */

#include "point_trajectory.h"

namespace velocity_controller
{

double distance(const point_t& p1, const point_t& p2)
{
  double dx = p1.x - p2.x;
  double dy = p2.y - p2.y;
  return sqrt(dx*dx + dy*dy);
}

void PointTrajectory::calculate_curvatures()
{
  curvatures.resize(points.size() - 1);
  for ( int i = 1; i<points.size()-1; ++i)
  {
    curvatures
  }

}

PointTrajectory::PointTrajectory(std::vector<geometry_msgs::Point32> points)
{
  this->points.reserve(points.size());
  length = 0;
  for( auto& p : points)
  {
    this->points.push_back( point_t(p.x, p.y) );
  }
  auto map_iterator = length_map.end();
  map_iterator = length_map.emplace_hint(map_iterator, 0.0, 0);
  for(int i = 1; i<this->points.size(); ++i)
  {
    length += distance(this->points[i-1], this->points[i]);
    map_iterator = length_map.emplace_hint(map_iterator, length, i);
  }

  //curvature calculation
  calculate_curvatures();
}


bool PointTrajectory::get_bound_indexes(double& length, int& prev_index, int& next_index)
{
  if (length > this->length)
    return false;

  auto prev_point = length_map.lower_bound(length);
  auto next_point = prev_point;
  ++next_point;
  prev_index = prev_point->first;
  next_index = prev_index + 1;

  length -= prev_point->second;
  return true;
}


bool PointTrajectory::get_point(double length, point_t& point)
{
  int prev_index, next_index;
  if ( !get_bound_indexes(length, prev_index, next_index) )
    return false;
  //calculate point by linear interpolation

  double k = length/ distance(points[prev_index], points[next_index]);
  double dx = points[next_index].x - points[prev_index].x;
  double dy = points[next_index].y - points[prev_index].y;

  point.x = points[prev_index] + dx * k;
  point.y = points[prev_index] + dy * k;

  return true;
}

double PointTrajectory::get_curvature(double length)
{
  int prev_index, next_index;
  if ( !get_bound_indexes(length, prev_index, next_index) )
    return 0;

}



PointTrajectory::~PointTrajectory()
{
  // TODO Auto-generated destructor stub
}

} /* namespace velocity_controller */
