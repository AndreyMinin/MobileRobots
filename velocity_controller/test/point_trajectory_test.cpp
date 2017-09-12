/*
 * point_trajectory_test.cpp
 *
 *  Created on: 9 июл. 2017 г.
 *      Author: aminin
 */


// gtest
#include <gtest/gtest.h>

#include "velocity_controller/point_trajectory.h"
#include <math.h>

using namespace velocity_controller;

void test_curvature(double c)
{
  std::vector<geometry_msgs::Point32> points;
  geometry_msgs::Point32 point;
  point.x = 30.0;
  point.y = -33.0;
  double v = 1.0;
  double dt = 0.2;
  double theta = -M_PI/6;

  //generate points
  const int N = 10;
  points.resize(N);
  for( int i = 0; i < N; ++i)
  {
    double fi = v*c*dt*i;
    double dx, dy;
    if ( fabs(fi) < 0.001 )
    {
      dx = v*dt*i;
      dy = 0;
    }
    else
    {
      dx = sin(fi)/c;
      dy = (1- cos(fi))/c;
    }
    points[i].x = point.x + dx*sin(theta) - dy*cos(theta);
    points[i].y = point.y + dx*cos(theta) + dy*sin(theta);
  }
  PointTrajectory trajectory(points);
  double length = trajectory.get_length();
  const int M = 3;
  double dl = length/M;
  for(int i = 1; i < M - 1; ++i)
  {
    EXPECT_NEAR(c, trajectory.get_curvature(dl*float(i)/float(M)), 0.001 );
  }
}


TEST(point_trajectory, line)
{
  test_curvature(0.1);
  test_curvature(0.01);
  test_curvature(-0.05);
  test_curvature(-0.1);
  test_curvature(0);
//  EXPECT_EQ(trajectory.get)


}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
