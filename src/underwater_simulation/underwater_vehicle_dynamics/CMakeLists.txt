cmake_minimum_required(VERSION 2.8.3)
project(underwater_vehicle_dynamics)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs nav_msgs geometry_msgs sensor_msgs tf tf_conversions 
#python_orocos_kdl
)

catkin_package(
    CATKIN_DEPENDS rospy std_msgs nav_msgs geometry_msgs sensor_msgs tf tf_conversions 
#python_orocos_kdl
)

install(PROGRAMS src/dynamics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS src/keyboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)

