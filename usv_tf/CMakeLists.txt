cmake_minimum_required(VERSION 2.8.3)
project(usv_tf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(world_tf_broadcaster src/world_tf_broadcaster.cpp)
add_executable(laser_tf_broadcaster src/laser_tf_broadcaster.cpp)
add_executable(tf_listener src/tf_listener.cpp)
target_link_libraries(world_tf_broadcaster ${catkin_LIBRARIES})
target_link_libraries(laser_tf_broadcaster ${catkin_LIBRARIES})
target_link_libraries(tf_listener ${catkin_LIBRARIES})

install(DIRECTORY scripts/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
   FILES_MATCHING PATTERN "*" 
   PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
               GROUP_READ GROUP_EXECUTE
               WORLD_READ WORLD_EXECUTE
)

install(DIRECTORY frames/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/frames
)
