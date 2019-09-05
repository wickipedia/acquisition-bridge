cmake_minimum_required(VERSION 2.8.3)
project(acquisition_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  duckietown_msgs # Every duckietown packages should use this.
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
