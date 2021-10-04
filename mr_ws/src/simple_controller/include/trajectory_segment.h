/*
 * traj.h
 *
 *  Created on: 4 июн. 2017 г.
 *      Author: aminin
 */

#ifndef SRC_TRAJECTORY_SEGMENT_H_
#define SRC_TRAJECTORY_SEGMENT_H_

#include <tf/LinearMath/Vector3.h>
#include <angles/angles.h>

namespace trajectory
{
/*!
 * part of trajectory with constant curvature
 * x0 - start point
 * v - start (unit) vector
 * length - length of trajectory segment
 * base class
 */
class TrajectorySegment
{
public:
  /// \ returns length of trajectory segment
  virtual double get_length() const = 0;
  /// \ returns curvature of segment
  virtual double get_curvature() const = 0;
  /// \ returns point of trajectory for the given length
  virtual tf::Vector3 get_point(double point_len) const = 0;
  /// \ returns flat orientation
  virtual double get_orientation(double point_len) const = 0;
  /// \ returns length of nearest trajectory point for the given point
  virtual double get_point_length(double x, double y) const = 0;
  /// \ returns distance to nearest trajectory point w r t direction (normal error)
  virtual double get_point_distance(double x, double y) const = 0;
  virtual ~TrajectorySegment(){}
};

class CircularSegment: public TrajectorySegment
{
protected:
  /// \ 1/R
  double curvature;
  /// \ start point
  tf::Vector3 x0;
  /// \ tangent vector to trajectory at start point
  tf::Vector3 v;
  /// \ trajectory length
  double length;
  /// \ angle from circular center to first point
  double start_angle;
  /// \ angle from start point to end point
  double finish_angle;
  /// \ center point
  tf::Vector3 center;

public:
  double get_length() const {return length;}

  double get_curvature() const { return curvature; }
  /// \ generates coordinates of point for point_len from trajectory start point
  tf::Vector3 get_point(double point_len) const
  {
    //arc case
    tf::Vector3 toCenter(-v.y(), v.x(), 0);
    tf::Vector3 center = x0 + toCenter/curvature;
    double angle = start_angle + point_len * curvature;
    return center + tf::Vector3(cos(angle), sin(angle), 0)/fabs(curvature);
  }

  double get_orientation(double point_len) const
  {
    return start_angle + point_len * curvature +
        ((curvature > 0.0) ? M_PI/2 : -M_PI/2);
  }

  /// \ returns length of trajectory for given point
  double get_point_length(double x, double y) const
  {
    //angle from center to point
    double point_angle = atan2(y - center.y(), x - center.x());
    point_angle = angles::normalize_angle(point_angle - start_angle);
    return point_angle / curvature;
  }

  double get_point_distance(double x, double y) const
  {
    //angle from center to point
    double point_angle = atan2(y - center.y(), x - center.x());
    double point_angle_rel = angles::normalize_angle(point_angle - start_angle);
    tf::Vector3 vr = v.rotate(tf::Vector3(0,0,1), point_angle_rel);
    tf::Vector3 delta = tf::Vector3(x, y, 0) - center - tf::Vector3(cos(point_angle)/ fabs(curvature), sin(point_angle)/ fabs(curvature),  0);
    return vr.cross(delta).z();
  }

  CircularSegment(double c, double x, double y, double vx, double vy, double l):
    curvature(c),
    x0(x, y, 0),
    v(vx, vy, 0),
    length(l),
    start_angle( (curvature > 0)? atan2(-v.x(), v.y()) :
                                  atan2(v.x(), -v.y()) ), //vector from center to first point (vy -vx) or (-vy, vx)
    finish_angle(start_angle + length * curvature),
    center(x0 + tf::Vector3(-v.y(), v.x(), 0)/curvature)
    { v.normalize(); }
};


class LinearSegment : public TrajectorySegment
{
protected:
  tf::Vector3 x0;
  tf::Vector3 v;
  double length;

public:

  double get_curvature() const { return 0; }

  double get_length() const {return length;}
  /// \ generates coordinates of point for point_len from trajectory start point
  tf::Vector3 get_point(double point_len) const
  {
    return x0 + point_len * v;
  }
  double get_orientation(double point_len) const {
    return atan2(v.y(), v.x());
  }
  /// \ returns length of trajectory for given point
  double get_point_length(double x, double y) const
  {
    //scalar product between v and x0-xy
    double dx = x- x0.x();
    double dy = y - x0.y();
    return v.x()*dx + v.y()*dy;
  }

  double get_point_distance(double x, double y) const
  {
    //vector product between v and x0-xy
    double dx = x- x0.x();
    double dy = y - x0.y();
    tf::Vector3 p ( dx, dy, 0.0 );
    return v.cross(p).z();
  }


  LinearSegment(double x, double y, double vx, double vy, double l) :
      x0(x, y, 0), v(vx, vy, 0), length(l)
  {}

};

}
#endif /* SRC_TRAJECTORY_SEGMENT_H_ */


