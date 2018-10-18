cmake_minimum_required(VERSION 2.8.3)
project(simple_planner)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_planner src/simple_planner.cpp src/planner.cpp src/planner.h)

target_link_libraries(simple_planner
  ${catkin_LIBRARIES}
)
