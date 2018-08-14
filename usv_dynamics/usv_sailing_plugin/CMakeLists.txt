cmake_minimum_required(VERSION 2.8.3)
project(usv_sailing_plugin)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gazebo_ros
  usv_wind_current
  usv_water_current
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES usv_sailing_plugin
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
 include
  ${catkin_INCLUDE_DIRS}  ${GAZEBO_INCLUDE_DIRS} 
)

add_library(usv_sailing_plugin SHARED src/usv_sailing_plugin.cpp include/usv_sailing_plugin/usv_sailing_plugin.h)
target_link_libraries(usv_sailing_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

install(
  TARGETS
    usv_sailing_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
