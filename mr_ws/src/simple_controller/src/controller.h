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
#include <nav_msgs/Path.h>
#include <list>
#include <memory>

#include "trajectory_segment.h"

namespace simple_controller
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
class Controller
{
protected:

  ros::NodeHandle nh;
  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_theta = 0.0;
  //time of robot coordinates update
  ros::Time robot_time;
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

  double current_linear_velocity = 0.0;
  double current_angular_velocity = 0.0;
  //discrete of publish trajectory
  double traj_dl;
  //length of published trajectory
  double traj_length;

  using Trajectory = std::list<TrajPtr>;
  /// \ container of trajectory segments
  std::list<TrajPtr> trajectory;

  nav_msgs::Path path;
  std::size_t nearest_point_index;

  /// \ current segment
  std::list<TrajPtr>::iterator current_segment;
  /// \ length of the current segment at the current point
  double current_segment_length = 0.0;

  ros::Subscriber pose_sub;
  ros::Subscriber odo_sub;
  ros::Subscriber path_sub;
  ros::Timer timer;
  ros::Publisher err_pub;
  ros::Publisher steer_pub;
  ros::Publisher path_pub;
  /// \ frame_id for coordinates of controller
  std::string world_frame_id;

  void on_timer(const ros::TimerEvent& event);
  void on_pose(const nav_msgs::OdometryConstPtr& odom);
  void on_path(const nav_msgs::Path& path);
  /*
   *@brief calculates feedback error for trajectory
   *@return feedback error
   */
  double cross_track_error();

  /// \ update robot pose to current time based on last pose and velocities
  void update_robot_pose(double dt);
  /*
   * \brief publishes trajectory as pointcloud message
   */
  void publish_trajectory();
  void on_odo(const nav_msgs::OdometryConstPtr& odom);
  void publish_error(double error);
  nav_msgs::Path create_path() const;
  std::size_t get_nearest_path_pose_index(int start_index,
                                          std::size_t search_len);

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
