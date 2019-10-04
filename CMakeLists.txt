cmake_minimum_required(VERSION 2.8.3)
project(ugv_trajectory_tracking)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  trajectory_msgs
  tf
  message_generation
)

catkin_package(
  INCLUDE_DIRS launch scripts
  LIBRARIES ugv_trajectory_tracking
  CATKIN_DEPENDS rospy std_msgs
  DEPENDS 
)
