cmake_minimum_required(VERSION 2.8.3)
project(usv_sim)

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


install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY meshes/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(DIRECTORY scenes/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scenes)

install(DIRECTORY terrain/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/terrain)

install(DIRECTORY urdf/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

install(DIRECTORY xacro/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/xacro)

install(DIRECTORY world/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/world)
