cmake_minimum_required(VERSION 2.8.3)
project(osg_interactive_markers)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  interactive_markers
)

find_package(osg_utils REQUIRED)
find_package(osg_markers REQUIRED)

catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS
  roscpp
  tf
  interactive_markers
DEPENDS
  osg_markers
  osg_utils
  libopenscenegraph
)

FIND_PACKAGE ( OpenSceneGraph COMPONENTS osgSim osgUtil osgDB osgFX osgGA osgTerrain osgViewer osgText osgWidget osgManipulator osg)

#link_directories(${VISP_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
include_directories(include ${osg_markers_INCLUDE_DIRS} ${osg_utils_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OSGMARKERS_INCLUDE_DIRS} ${OSGUTILS_INCLUDE_DIRS})
add_library(${PROJECT_NAME}
  src/interactive_marker_client.cpp
  src/interactive_marker_display.cpp
  src/interactive_marker.cpp
  src/interactive_marker_control.cpp
  src/draggers.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES} ${OSGUTILS_LIBRARIES} ${OSGMARKERS_LIBRARIES} -lGLU )

add_executable(osg_interactive_markers_demo examples/osg_interactive_markers_demo.cpp)
target_link_libraries(osg_interactive_markers_demo ${PROJECT_NAME} ${catkin_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES} ${osg_markers_LIBRARIES} ${osg_utils_LIBRARIES} ${OSGUTILS_LIBRARIES} ${OSGMARKERS_LIBRARIES} -lGLU )

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


configure_file(osgInteractiveMarkersConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/lib/${CMAKE_PROJECT_NAME}Config.cmake @ONLY)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/lib/${CMAKE_PROJECT_NAME}Config.cmake 
        DESTINATION share/${CMAKE_PROJECT_NAME} )

install(
TARGETS
osg_interactive_markers_demo 
${PROJECT_NAME}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
