/*
 * velocity_controller.h
 *
 *  Created on: 12 июн. 2017 г.
 *      Author: aminin
 */

#ifndef SRC_VELOCITY_CONTROLLER_H_
#define SRC_VELOCITY_CONTROLLER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include "velocity_controller/point_trajectory.h"

namespace velocity_controller
{

struct velocity_plan_point // motion parameters in current point
{
  double vel;
  double acc;
  double jerk;
  double len;
  velocity_plan_point():vel(0),acc(0),jerk(0),len(0){}
};

using velocity_plan_t = std::list<velocity_plan_point>;

enum Blocker
{
  NoBlock,
  TrajectoryFinish,
  MaxVelExceeded,
  MaxAccExceeded,
  MaxDccExceeded,
  Collision,
  MaxCentripetalAccExceeded,
  MaxAngularAccExceeded
};

class VelocityController
{
protected:
  ros::NodeHandle nh;
  double dt;
  ros::Timer timer;
  ros::Subscriber traj_sub;
  ros::Subscriber map_sub;
  ros::Subscriber odo_sub;
  ros::Publisher cmdvel_pub;

  double max_velocity;
  double max_acc;
  double max_dcc;
  double jerk;
  double chassys_width;
  double chassys_length;
  double max_centripetal_acc;
  double max_angular_acc;


  double current_velocity = 0.0;
  double cmd_velocity = 0.0;
  double acc = 0.0;
  double long_run_time;

  PointTrajectory trajectory;


  velocity_plan_t velocity_plan;
  velocity_plan_t dcc_plan;

  Blocker check_point(const velocity_plan_point& point);
  velocity_plan_point generate_point(velocity_plan_point& point, double t);
  bool generate_points(int n, velocity_plan_t& plan, double jerk);
  // remind point - point left from previous generation and also point to save middle state before next generation
  // remind time - is value of previous time quantum for remind point
  bool generate_points(double plan_time, velocity_plan_t& plan, double jerk,
                       double& remind_time, velocity_plan_point& remind_point);
  bool check_stop(velocity_plan_t& stop_plan);
  bool check_longrun(const velocity_plan_t& test_plan);

  bool update_velocity_plan();
  void on_timer(const ros::TimerEvent& event);
  void on_trajectory(const sensor_msgs::PointCloudConstPtr& msg);
  void on_map(const nav_msgs::OccupancyGridConstPtr& msg);
  void on_odo(const nav_msgs::OdometryConstPtr& msg);
public:
  VelocityController();
  virtual ~VelocityController();
};

} /* namespace velocity_controller */

#endif /* SRC_VELOCITY_CONTROLLER_H_ */
