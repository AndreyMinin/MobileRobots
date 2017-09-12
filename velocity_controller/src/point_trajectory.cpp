/*
 * point_trajectory.cpp
 *
 *  Created on: 2 июл. 2017 г.
 *      Author: aminin
 */

#include "velocity_controller/point_trajectory.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>


namespace velocity_controller
{

double distance(const point_t& p1, const point_t& p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return sqrt(dx*dx + dy*dy);
}

double PointTrajectory::curvature(const point_t& p1, const point_t& p2, const point_t& p3)
{
  // find circle by three points
  // (cx - x)^2 + (cy - y)^2 = R => 2(xi-xj)cx + 2(yi-yj)cy = xi^2 - xj^2 + yi^2 - yj^2
  Eigen::Matrix2d M;
  M << 2 * (p2.x - p1.x), 2 * (p2.y - p1.y),
      2 * (p2.x - p3.x), 2 * (p2.y - p3.y);


  std::cout<<"M = "<<M<<" det = "<<M.determinant()<<std::endl;
  if ( fabs(M.determinant()) < 0.0000001 )
  {
    return 0;
  }

  double p1x2 = p1.x*p1.x;
  double p1y2 = p1.y*p1.y;
  double p2x2 = p2.x*p2.x;
  double p2y2 = p2.y*p2.y;
  double p3x2 = p3.x*p3.x;
  double p3y2 = p3.y*p3.y;

  Eigen::Vector2d B( p2x2 - p1x2 + p2y2 - p1y2,
                     p2x2 - p3x2 + p2y2 - p3y2);
  // find C = (cx, cy)T
  Eigen::Vector2d C = M.inverse() * B;
  // find radius from first point
  double dx = C[0] - p1.x;
  double dy = C[1] - p1.y;

  // signum of curvature
  // signum of cross product p1p2 and p1c
  double sign = (p2.x - p1.x)*dy - (p2.y - p1.y)*dx;

  return copysign(1.0 /sqrt(dx*dx + dy*dy), sign);
}

void PointTrajectory::calculate_curvatures()
{
  curvatures.resize(points.size() - 1);
  for (int i = 1; i<points.size()-1; ++i)
  {
    curvatures[i-1] = curvature(points[i-1], points[i], points[i+1]);
    std::cout<<" "<<curvatures[i-1]<<std::endl;
  }
  std::cout<<std::endl;
}

PointTrajectory::PointTrajectory(std::vector<geometry_msgs::Point32> points)
{
  this->points.reserve(points.size());
  length = 0;
  for(auto& p : points)
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

  auto prev_point = length_map.lower_bound(length); //returns iterator to point with equal or greater length
  prev_point--; //repoint it to point with lesser length
  auto next_point = prev_point;
  ++next_point;
  prev_index = prev_point->first;
  next_index = prev_index + 1;

  assert(length < next_point->first);
  assert(length > prev_point->first);
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

  point.x = points[prev_index].x + dx * k;
  point.y = points[prev_index].y + dy * k;

  return true;
}

double PointTrajectory::get_curvature(double length)
{
  int prev_index, next_index;
  if ( !get_bound_indexes(length, prev_index, next_index) )
    return 0;
  assert(prev_index < curvatures.size());
  return curvatures[prev_index];
}



PointTrajectory::~PointTrajectory()
{
  // TODO Auto-generated destructor stub
}

} /* namespace velocity_controller */
