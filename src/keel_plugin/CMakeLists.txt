cmake_minimum_required(VERSION 2.8.3)
project(keel_plugin)
#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs

    gazebo_ros
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES liftdragww
  CATKIN_DEPENDS roscpp std_msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories( include
  ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} 
)

add_library(keel_plugin SHARED src/keel_plugin.cpp include/keel_plugin/keel_plugin.h)
target_link_libraries(keel_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

install(
  TARGETS
    keel_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
