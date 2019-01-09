cmake_minimum_required(VERSION 2.8.3)
project(wind_current)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  rospy
  std_msgs
)

find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)

add_service_files(
  FILES
    GetSpeed.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wind_current
  CATKIN_DEPENDS message_runtime std_msgs
  DEPENDS 
    nav_msgs
    geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY maps/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY scripts/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
   FILES_MATCHING PATTERN "*" 
   PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
               GROUP_READ GROUP_EXECUTE
               WORLD_READ WORLD_EXECUTE
)
