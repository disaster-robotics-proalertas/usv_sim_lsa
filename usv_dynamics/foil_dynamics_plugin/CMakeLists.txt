cmake_minimum_required(VERSION 2.8.3)
project(foil_dynamics_plugin)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gazebo_ros
  wind_current
  water_current
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES foil_dynamics_plugin
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
 include
  ${catkin_INCLUDE_DIRS}  ${GAZEBO_INCLUDE_DIRS} 
)

add_library(foil_dynamics_plugin SHARED src/foil_dynamics_plugin.cpp include/foil_dynamics_plugin/foil_dynamics_plugin.h)
target_link_libraries(foil_dynamics_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

install(
  TARGETS
    foil_dynamics_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
