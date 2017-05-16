cmake_minimum_required(VERSION 2.8.3)
project(osg_utils)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS
  roscpp
  tf
DEPENDS
  libopenscenegraph
)

#link_directories(${VISP_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} )
add_library(${PROJECT_NAME}
  src/osg_utils.cpp
  src/frame_manager.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

configure_file(osgUtilsConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/lib/${CMAKE_PROJECT_NAME}Config.cmake @ONLY)

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
