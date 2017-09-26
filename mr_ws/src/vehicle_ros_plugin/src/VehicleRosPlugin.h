/*
 * VehicleRosPlugin.h
 *
 *      Author: aminin
 */

#ifndef SRC_VEHICLEROSPLUGIN_H_
#define SRC_VEHICLEROSPLUGIN_H_

/*
 * \file VehicleRosPlugin.h
 *
 * \brief Ackerman steering chassis plugin for gazebo. Based on gazebo_ros_diff_drive
 *
 *
 */

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>


#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <thread>
#include <mutex>

namespace gazebo
{

class VehicleRosPlugin: public ModelPlugin
{
  /*!
   * \brief enum for driven wheels variants
   */
  enum Wheels
  {
    FRONT,
    REAR,
    ALL
  };
  /*!
   * \brief all movable joints
   */
  enum JointsEnum
  {
    FRONT_LEFT_WHEEL,
    FRONT_RIGHT_WHEEL,
    REAR_LEFT_WHEEL,
    REAR_RIGHT_WHEEL,
    FRONT_LEFT_STEERING,
    FRONT_RIGHT_STEERING,
    JOINTS_NUMBER
  };


public:
  VehicleRosPlugin();
  virtual ~VehicleRosPlugin();
  /*!
   * \brief loads parameters from sdf file
   * command_topic - topic name for geometry_msgs::Twist control command, (cmd_vel) <br>
   * -odo_topic - topic name for publishing odo messages for vehicle, (odom) <br>
   * -odom_frame - base frame_id for odo message (odom) <br>
   * -base_frame - vehicle frame_id for odo message (base_link) <br>
   * -steering_angle_topic - topic for steering angle commands as std_msgs::Float32, (steering_angle) <br>
   * -velocity_topic - topic for setting velocity as std_msgs::Float32 (velocity) <br>
   * -publishJointState - bool, true for publishing joint states (true) <br>
   * -wheel_separation - distance between wheels in pair (2.0) <br>
   * -wheel_diameter - diameter of every wheel (0.5) <br>
   * -axes_separation - distance between wheel pairs (3.0) <br>
   * -vehicle_accel - acceleration for velocity changing , 0.0 - for unbounded acceleration <br>
   * -wheel_torque - gazebo max torque value for driven wheels <br>
   * -curvature_rate - rate of change for wheel turn, 0.0 - for immediate change <br>
   * -steering_torque - gazebo max torque value for turn wheel joints <br>
   * -update_rate - plugin update rate <br>
   * -min_radius - minimum turn radius of wheel <br>
   * -encoder_wheels - "front" or "rear" wheels for odometry measurements <br>
   * -drive_wheels - "front" or "rear" or "all" driven wheels <br>
   * -front_left_wheel_joint <br>
   * -front_right_wheel_joint <br>
   * -rear_left_wheel_joint <br>
   * -rear_right_wheel_joint <br>
   * -front_left_steering_joint <br>
   * -front_right_steering_joint - names of controlled joints <br>
   *
   */
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  /*!
   * \brief resets model in to initial state
   */
  void Reset();

protected:
  /*!
   * \brief main update function
   */
  virtual void UpdateChild();
  /*!
   * \brief finalize function
   */
  virtual void FiniChild();

private:
  /*!
   * \brief publishes odometry for ROS
   */
  void publishOdometry();
  /*!
   * \brief copies curvature and velocity command from common variables
   * \param c, v curvature and velocities
   */
  void getVehicleCmd(double& c, double& v);

  /*!
   * \brief limits velocity according to acceleration bounds
   */
  void limitVelocityCmd(double& velocity, double dt);
  /*!
   * \brief limits curvature according to rate bounds
   */
  void limitCurvatureCmd(double& curvature, double dt);
  ///!calculates joints commands
  void calculateJoints(double curvature, double velocity);
  /*!
   * \brief calculates front wheel angles values for given curvature and velocity
   */
  void calculateFrontAngles(double curvature, double velocity);
  /*!
   * \brief calculates front wheels rate
   */
  void calculateFrontJoints(double curvature, double velocity);
  /*!
   * \brief calculates rear wheels rate
   */
  void calculateRearJoints(double curvature, double velocity);

  /*!
   * \brief sets calculated values for every joint
   */
  void setJoints();

  void publishWheelJointStates();
  /*!
   * update odometry based on rear wheels
   */
  void UpdateRearOdometryEncoder();
  /*!
   * updates odometry based on front wheels
   */
  void UpdateFrontOdometryEncoder();

  GazeboRosPtr gazebo_ros;
  physics::ModelPtr parent;
  event::ConnectionPtr update_connection;

  double wheel_separation; //between left and right
  double axes_separation; //between forward and backward
  double min_radius;
  double wheel_diameter;
  double wheel_radius;
  double wheel_torque;
  double steering_torque;

  double curvature_rate;
  double vehicle_accel;
  //measured values
  double vehicle_velocity;
  double vehicle_curvature;
  //last values set to joints
  double set_velocity;
  double set_curvature;
  std::vector<physics::JointPtr> joints;

  //ROS STUFF
  ros::NodeHandlePtr node;
  nav_msgs::Odometry odom_msg;
  ros::Publisher odo_pub;

  ros::Subscriber cmd_vel_sub;
  ros::Subscriber steering_angle_sub;
  ros::Subscriber velocity_sub;

  sensor_msgs::JointState joint_state_msg;
  ros::Publisher joint_state_pub;

  ///! mutex for command exchange between ros and plugin threads
  std::mutex lock;

  std::string robot_namespace;
  std::string command_topic;
  std::string steering_angle_topic;
  std::string velocity_topic;
  std::string odo_topic;


  ///!custom Callback Queue
  ros::CallbackQueue ros_queue;
  ///! ROS thread
  std::thread ros_queue_thread;
  ///! ROS thrread function
  void QueueThread();

  //ros callbacks
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
  void steeringAngleCallback(const std_msgs::Float32ConstPtr& angle_cmd);
  void velocityCallback(const std_msgs::Float32ConstPtr& vel_cmd);

  ///! common variables for threads
  double cmd_velocity;
  double cmd_curvature;
  //cmd for wheels
  double cmd_rate[4];
  //cmd for turn wheels
  double cmd_left_angle;
  double cmd_right_angle;
  bool alive;

  double update_rate;
  double update_period;
  common::Time last_update_time;
  //which wheels are encoder is on
  Wheels encoder_wheels;
  //which wheels are driven by motors
  Wheels drive_wheels;
  geometry_msgs::Pose2D pose_encoder;
  common::Time last_odom_update;

  bool pulishJointState;


};

} /* namespace gazebo */

#endif /* SRC_VEHICLEROSPLUGIN_H_ */
