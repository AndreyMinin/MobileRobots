/*
 * odo2tf.cpp
 *
 *  Created on: 14 июн. 2017 г.
 *      Author: aminin
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <memory>

std::unique_ptr<tf::TransformBroadcaster> tf_caster_ptr;

void on_odo(const nav_msgs::OdometryConstPtr msg)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  tf::Vector3 v;
  tf::pointMsgToTF(msg->pose.pose.position, v);
  tf::Transform transf(q, v);
  tf::StampedTransform stamped_transf(transf, msg->header.stamp, msg->header.frame_id, msg->child_frame_id);
  tf_caster_ptr->sendTransform(stamped_transf);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "odo2tf");
  tf_caster_ptr.reset(new tf::TransformBroadcaster );
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("odo", 1, on_odo);
  ros::spin();
  return 0;
}



