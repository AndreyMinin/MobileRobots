/*
 * point_trajectory.h
 *
 *  Created on: 2 июл. 2017 г.
 *      Author: aminin
 */

#ifndef INCLUDE_VELOCITY_CONTROLLER_POINT_TRAJECTORY_H_
#define INCLUDE_VELOCITY_CONTROLLER_POINT_TRAJECTORY_H_


#include <vector>
#include <map>
#include <geometry_msgs/Point32.h>

namespace velocity_controller
{

struct point_t
{
  double x, y;
  point_t(double _x, double _y):
    x(_x),y(_y){}
};

class PointTrajectory
{
protected:
  std::vector<point_t> points;
  //map from length to index of point
  std::map<double, int> length_map;
  double length;
  std::vector<double> curvatures;
  //returns true if length is in trajectory
  //prev next - bound indexes
  //length is trunkated to length from prev point
  bool get_bound_indexes(double& length, int& prev, int& next);
  //calculation by second order
  double curvature(const point_t& p1, const point_t& p2, const point_t& p3);
  void calculate_curvatures();
public:

  bool get_point(double length, point_t& point);
  double get_curvature(double length);
  double get_length() { return this->length; }
  PointTrajectory(std::vector<geometry_msgs::Point32> points);
  PointTrajectory():length(0){}
  virtual ~PointTrajectory();
};

} /* namespace velocity_controller */

#endif /* INCLUDE_VELOCITY_CONTROLLER_POINT_TRAJECTORY_H_ */
