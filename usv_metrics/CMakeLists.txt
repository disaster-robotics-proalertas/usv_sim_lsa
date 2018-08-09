cmake_minimum_required(VERSION 2.8.3)
project(usv_metrics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES usv_navigation
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

