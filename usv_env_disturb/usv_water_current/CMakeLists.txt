cmake_minimum_required(VERSION 2.8.3)
project(usv_water_current)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  rosgraph_msgs
  message_generation
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
  roscpp
  std_msgs
  rosgraph_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES usv_water_current
  CATKIN_DEPENDS message_runtime roscpp std_msgs
  DEPENDS 
    roscpp
    nav_msgs
    geometry_msgs
    rosgraph_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${SDL_INCLUDE_DIR}
  ${SDL_IMAGE_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${YAMLCPP_LIBRARIES}
   ${SDL_LIBRARY}
   ${SDL_IMAGE_LIBRARIES}
)

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY maps/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps)

install(DIRECTORY scripts/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
   FILES_MATCHING PATTERN "*" 
   PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
               GROUP_READ GROUP_EXECUTE
               WORLD_READ WORLD_EXECUTE
)

