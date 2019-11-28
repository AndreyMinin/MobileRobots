cmake_minimum_required(VERSION 2.8.3)
project(feature_matcher)

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
find_package(cmake_modules REQUIRED)

find_package(Eigen REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(feature_matcher src/feature_matcher.cpp
							   src/matcher.h
							   src/matcher.cpp)

target_link_libraries(feature_matcher
  ${catkin_LIBRARIES}
)
