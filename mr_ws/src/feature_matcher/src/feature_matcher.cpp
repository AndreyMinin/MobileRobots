#include <ros/ros.h>
#include "matcher.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "matcher");
  ros::NodeHandle nh("~");

  Matcher matcher(nh);

  ros::spin();
  return 0;
}



