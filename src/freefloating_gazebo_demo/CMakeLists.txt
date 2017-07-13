cmake_minimum_required(VERSION 2.8.3)
project(freefloating_gazebo_demo)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
)

catkin_package(
 LIBRARIES ${PROJECT_NAME}
 DEPENDS
    roscpp
    sensor_msgs
    nav_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

# demo node
add_executable(freefloating_gazebo_demo src/freefloating_gazebo_demo.cpp)
target_link_libraries(freefloating_gazebo_demo ${catkin_LIBRARIES})

install(
  TARGETS
    freefloating_gazebo_demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY objects/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/objects)

install(DIRECTORY scenes/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scenes)

install(DIRECTORY scripts/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

install(DIRECTORY terrain/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/terrain)

install(DIRECTORY urdf/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
