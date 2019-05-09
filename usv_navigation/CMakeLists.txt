cmake_minimum_required(VERSION 2.8.3)
project(usv_navigation)

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

install(DIRECTORY bags/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bags)

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY maps/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps)

install(DIRECTORY param/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param)

install(DIRECTORY scripts/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
   FILES_MATCHING PATTERN "*" 
   PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
               GROUP_READ GROUP_EXECUTE
               WORLD_READ WORLD_EXECUTE
)
