/*
 * controller_node.cpp
 *
 *  Created on: 29 апр. 2017 г.
 *      Author: aminin
 */

#include <ros/ros.h>


#include <dynamic_reconfigure/server.h>
#include <simple_controller/ControllerDynReconfConfig.h>

#include "controller.h"

void reconfigure_callback( simple_controller::Controller& ctrl,
                          simple_controller::ControllerDynReconfConfig& config,
                          uint32_t level)
{
  ROS_INFO_STREAM("Reconfigure "<<config.proportional<<" "<<config.differential<<" "<<config.integral);
  ctrl.reset(config.proportional, config.differential, config.integral);
}

volatile bool odo_received = false;
ros::Subscriber odo_sub;
void on_odo(const nav_msgs::OdometryConstPtr& odom)
{
  odo_received = true;
  odo_sub.shutdown();
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simple_controller");
  ros::NodeHandle nh("~/simple_controller");

  //try to receive first message from robot before starting controller
  //this stuff was introduced for gazebo
  odo_sub = nh.subscribe("odom", 1, on_odo);
  while ( ros::ok() && !odo_received )
  {
    ros::spinOnce();
  }
  ROS_DEBUG_STREAM("odo received");

  dynamic_reconfigure::Server<simple_controller::ControllerDynReconfConfig> server(nh);
  dynamic_reconfigure::Server<simple_controller::ControllerDynReconfConfig>::CallbackType f;

  simple_controller::Controller Cntrl;

  f = boost::bind( &reconfigure_callback, boost::ref(Cntrl), _1, _2);

//  simple_controller::ControllerConfig default_config;
//  default_config.differential = Cntrl.get_d_factor();
//  default_config.integral = Cntrl.get_i_factor();
//  default_config.proportional = Cntrl.get_p_factor();
//  server.setConfigDefault(default_config);

  server.setCallback(f);

  ros::spin();
  return 0;

}

