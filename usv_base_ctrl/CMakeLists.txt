cmake_minimum_required(VERSION 2.8.3)
project(usv_base_ctrl)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES usv_base_ctrl
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

install(DIRECTORY scripts/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
   FILES_MATCHING PATTERN "*" 
   PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
               GROUP_READ GROUP_EXECUTE
               WORLD_READ WORLD_EXECUTE
)
