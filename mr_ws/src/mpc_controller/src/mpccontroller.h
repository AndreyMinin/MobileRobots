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
#include <memory>
#include <acado/acado_toolkit.hpp>
#include <tf/tf.h>
#include "trajectory_segment.h"
#include "mpc.h"


namespace mpc_controller
{

using TrajPtr = std::shared_ptr<trajectory::TrajectorySegment>;

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
class MPCController
{
protected:

  ros::NodeHandle nh;
  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_theta = 0.0;
  //time of robot coordinates update
  ros::Time robot_time;


  ///\ circle params
  double radius;
  ///\ second circle center
  double  cy;

  // robot params
  double wheel_base;
  double max_steer_angle;
  double max_steer_rate;
  double max_velocity;
  double max_acc;

  double current_linear_velocity = 0.0;
  double current_angular_velocity = 0.0;
  double current_curvature = 0.0;
  double current_angle = 0.0;
  double control_dt;
  //discrete of publish trajectory
  double traj_dl;
  //length of published trajectory
  double traj_length;

  using Trajectory = std::list<TrajPtr>;
  /// \ container of trajectory segments
  Trajectory trajectory;

  /// \ current segment
  Trajectory::iterator current_segment;
  /// \ length of the current segment at the current point
  double current_segment_length = 0.0;

  ros::Subscriber pose_sub;
  ros::Subscriber odo_sub;
  ros::Timer timer;
  ros::Publisher err_pub;
  ros::Publisher steer_pub;
  ros::Publisher vel_pub;
  ros::Publisher traj_pub;
  ros::Publisher poly_pub;
  ros::Publisher mpc_traj_pub;
  /// \ frame_id for coordinates of controller
  std::string world_frame_id;

  double cmd_vel = 0;
  double cmd_acc = 0;
  double cmd_steer_angle = 0;
  double cmd_steer_rate = 0;
  std::vector<double> mpc_x,mpc_y;

  void on_timer(const ros::TimerEvent& event);
  void on_pose(const nav_msgs::OdometryConstPtr& odom);
  /*
   *@brief calculates feedback error for trajectory
   *@return feedback error
   */
  double cross_track_error();

  /// \ returns iterator to segment and current length  of trajectory belonging to current position
  void get_segment(std::list<TrajPtr>::iterator& traj_it, double& len);

  /// \ update robot pose to current time based on last pose and velocities
  void update_robot_pose(double dt);
  /*
   * \brief publishes trajectory as pointcloud message
   */
  void publish_trajectory();
  void on_odo(const nav_msgs::OdometryConstPtr& odom);
  void update_trajectory_segment();
  void publish_error(double error);

  std::vector<tf::Vector3> control_points;
  double control_points_dl = 2.0;
  std::size_t control_points_num = 6;
  double mpc_steps ;
  double mpc_dt;
  // coefs for y = f(x) for control points
  std::vector<double> control_coefs;

  MPC mpc;

  tf::Transform robot2world;

  void update_control_points();
  // converts control points into robot coordinate frame
  void convert_control_points();
  void calculate_control_coefs();
  double polyeval(double x);
  void apply_control();
  void publish_poly();
  void publish_mpc_traj(std::vector<double>& x, std::vector<double>& y);
public:
  void reset();
  MPCController(const std::string& ns = "mpc_controller");
  virtual ~MPCController();
};

} /* namespace simple_controller */

#endif /* SRC_CONTROLLER_H_ */
