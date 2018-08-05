cmake_minimum_required(VERSION 2.8.3)
project(osg_markers)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  roslib
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS
  roscpp
  tf
  roslib
DEPENDS
  libopenscenegraph
)

#link_directories(${VISP_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} )
add_library(${PROJECT_NAME}
  src/marker_base.cpp
  src/shape_marker.cpp
  src/text_view_facing_marker.cpp
  src/triangle_list_marker.cpp
  src/arrow_marker.cpp
  src/mesh_resource_marker.cpp
)

target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

configure_file(osgMarkersConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/lib/${CMAKE_PROJECT_NAME}Config.cmake @ONLY)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/lib/${CMAKE_PROJECT_NAME}Config.cmake 
        DESTINATION share/${CMAKE_PROJECT_NAME} )

install(
TARGETS
${PROJECT_NAME}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
