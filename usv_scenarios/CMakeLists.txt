cmake_minimum_required(VERSION 2.8.3)
project(usv_scenarios)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
    move_base
)

catkin_package(
 LIBRARIES ${PROJECT_NAME}
 DEPENDS
    roscpp
    sensor_msgs
    nav_msgs
    move_base
)

include_directories(${catkin_INCLUDE_DIRS})


install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY meshes/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(DIRECTORY scenes/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scenes)

install(DIRECTORY terrain/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/terrain)

install(DIRECTORY world/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/world)

install(DIRECTORY scripts/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
