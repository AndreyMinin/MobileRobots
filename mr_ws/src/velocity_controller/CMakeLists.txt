cmake_minimum_required(VERSION 2.8.3)
project(velocity_controller)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(velocity_controller src/velocity_controller.cpp)

target_link_libraries(velocity_controller
  ${catkin_LIBRARIES}
)

add_executable(velocity_test src/velocity_test.cpp)

target_link_libraries(velocity_test
  ${catkin_LIBRARIES}
)
