cmake_minimum_required(VERSION 2.8.3)
project(usv_sim_rviz)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES usv_rviz
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY rviz/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
