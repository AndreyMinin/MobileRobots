/*
 * Controller.h
 *
 *  Created on: 30 апр. 2017 г.
 *      Author: aminin
 */

#ifndef SRC_CONTROLLER_H_
#define SRC_CONTROLLER_H_


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <list>
#include "trajectory.h"
#include <memory>

namespace simple_controller
{

using TrajPtr = std::shared_ptr<trajectory::Trajectory>;

/*!
 *\brief robot controller
 * controls following along defined trajectory via simple pid regulator
 * angular_velocity = pid(error)
 * error is distance to trajectory
 * trajectory is list of angular and linear segments, saved as pointers to base class Trajectory
 * Trajectory is cycled
 * feedback from robot is received via ground_truth callback (real position of robot)
 * during control future trajectory is published for velocity controller
 */
class Controller
{
protected:

  ros::NodeHandle nh;
  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_theta = 0.0;
  double p_factor;
  double d_factor;
  double i_factor;
  double max_antiwindup_error;
  double error_integral;
  double last_error;

  ///\ circle params
  double radius;
  ///\ second circle center
  double  cy;

  double max_curvature;

  double current_velocity = 0.0;
  //discrete of publish trajectory
  double traj_dl;
  //length of published trajectory
  double traj_length;

  using Trajectory = std::list<TrajPtr>;
  /// \ container of trajectory segments
  std::list<TrajPtr> trajectory;

  /// \ current segment
  std::list<TrajPtr>::iterator current_segment;
  /// \ length of the current segment at the current point
  double current_segment_length = 0.0;

  ros::Subscriber pose_sub;
  ros::Subscriber odo_sub;
  ros::Timer timer;
  ros::Publisher err_pub;
  ros::Publisher steer_pub;
  ros::Publisher traj_pub;

  void on_timer(const ros::TimerEvent& event);
  void on_pose(const nav_msgs::OdometryConstPtr& odom);
  /*
   *@brief calculates feedback error for trajectory
   *@return feedback error
   */
  double cross_track_error();

  /// \ returns iterator to segment and current length  of trajectory belonging to current position
  void get_segment(std::list<TrajPtr>::iterator& traj_it, double& len);

  /*
   * \brief publishes trajectory as pointcloud
   */
  void publish_trajectory();
  void on_odo(const nav_msgs::OdometryConstPtr& odom);
  void update_trajectory_segment();

public:
  double get_p_factor(){ return p_factor; }
  double get_d_factor(){ return d_factor; }
  double get_i_factor(){ return i_factor; }
  void reset();
  void reset(double p, double d, double i );
  Controller(const std::string& ns = "simple_controller");
  virtual ~Controller();
};

} /* namespace simple_controller */

#endif /* SRC_CONTROLLER_H_ */