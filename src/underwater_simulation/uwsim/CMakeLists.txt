cmake_minimum_required(VERSION 2.8.3)
project(uwsim)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_transport
  interactive_markers
  kdl_parser
  nav_msgs
  pcl_ros
  pluginlib
  resource_retriever
  robot_state_publisher
  roscpp
  sensor_msgs
  tf
  underwater_sensor_msgs
  urdf
  xacro
)

find_package(uwsim_osgocean REQUIRED)
find_package(uwsim_osgworks REQUIRED)
find_package(uwsim_osgbullet REQUIRED)
find_package(uwsim_bullet REQUIRED)
find_package(osg_utils REQUIRED)
find_package(osg_interactive_markers REQUIRED)
find_package(osg_markers REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    uwsim
  CATKIN_DEPENDS
    geometry_msgs
    image_transport
    interactive_markers
    kdl_parser
    nav_msgs
    pcl_ros
    pluginlib
    resource_retriever
    robot_state_publisher
    roscpp
    sensor_msgs
    tf
    underwater_sensor_msgs
    urdf
  DEPENDS
    boost
    libopenscenegraph
    libxml++-2.6
    opengl
    osg_utils
    osg_markers
    osg_interactive_markers
    uwsim_bullet
    uwsim_osgocean
    uwsim_osgworks
)

find_package(Boost REQUIRED thread program_options system filesystem)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules;{CMAKE_MODULE_PATH}")

FIND_PACKAGE ( OpenSceneGraph COMPONENTS osgSim osgUtil osgDB osgFX osgGA osgTerrain osgViewer osgText osgWidget osg osgParticle osgManipulator)

SET(UWSIM_ROOT_PATH "${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}")

CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/include/uwsim/SimulatorConfig.h.cmake
  ${PROJECT_SOURCE_DIR}/include/uwsim/SimulatorConfig.h
)

find_package(PkgConfig)
pkg_check_modules(LIBXML++ REQUIRED libxml++-2.6)

INCLUDE_DIRECTORIES(include ${BULLET_INCLUDE_DIRS} ${OSGWORKS_INCLUDE_DIR} ${OSGBULLET_INCLUDE_DIR} ${OSGOCEAN_INCLUDE_DIRS} ${LIBXML++_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OSGUTILS_INCLUDE_DIRS} ${OSGMARKERS_INCLUDE_DIRS} ${OSGINTERACTIVEMARKERS_INCLUDE_DIRS} ${osg_interactive_markers_INCLUDE_DIRS})

link_directories(${orocos_kdl_LIBRARY_DIRS})

# this disables the warnings that come mostly from osg
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-ignored-qualifiers")
# Needed for kinetic
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-ignored-qualifiers -Wall")

add_library(uwsim 
src/osgOceanScene.cpp
src/SkyDome.cpp
src/SphereSegment.cpp
src/KinematicChain.cpp
src/URDFRobot.cpp
src/SimulatedIAUV.cpp
src/VirtualCamera.cpp
src/VirtualRangeSensor.cpp
src/HUDCamera.cpp
src/ConfigXMLParser.cpp
src/UWSimUtils.cpp
src/ROSInterface.cpp
src/ObjectPicker.cpp
src/SceneBuilder.cpp
src/ViewBuilder.cpp
src/SimulatedDevices.cpp 
src/InertialMeasurementUnit.cpp
src/PressureSensor.cpp
src/DVLSensor.cpp
src/GPSSensor.cpp
src/MultibeamSensor.cpp
src/VirtualSLSProjector.cpp
src/BulletPhysics.cpp
src/PhysicsBuilder.cpp
src/ROSSceneBuilder.cpp
)

add_library(uwsim_plugins_simdev 
src/SimDev_Echo.cpp
src/ForceSensor.cpp
src/DredgeTool.cpp
)
target_link_libraries(uwsim_plugins_simdev ${catkin_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES})


target_link_libraries(uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OSGOCEAN_LIBRARIES} ${OSGWORKS_LIBRARIES} ${OSGBULLET_LIBRARIES} ${BULLET_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES} ${LIBXML++_LIBRARIES} -lGLU)
add_dependencies(uwsim underwater_sensor_msgs_gencpp)


add_executable(uwsim_binary  src/main.cpp)
target_link_libraries(uwsim_binary uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OSGOCEAN_LIBRARIES} ${OSGWORKS_LIBRARIES} ${OSGBULLET_LIBRARIES} ${BULLET_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES} ${LIBXML++_LIBRARIES} ${OSGUTILS_LIBRARIES} ${OSGINTERACTIVEMARKERS_LIBRARIES} ${osg_interactive_markers_LIBRARIES} ${OSGMARKERS_LIBRARIES} -lGLU )

add_executable(makeVehicleSurvey interface_examples/makeVehicleSurvey.cpp)
target_link_libraries(makeVehicleSurvey uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(setJointPosition interface_examples/setJointPosition.cpp)
target_link_libraries(setJointPosition uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(setJointVelocity interface_examples/setJointVelocity.cpp)
target_link_libraries(setJointVelocity uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(setVehiclePosition interface_examples/setVehiclePosition.cpp)
target_link_libraries(setVehiclePosition uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(setVehiclePose interface_examples/setVehiclePose.cpp)
target_link_libraries(setVehiclePose uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(setVehicleVelocity interface_examples/setVehicleVelocity.cpp)
target_link_libraries(setVehicleVelocity uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(setVehicleTwist interface_examples/setVehicleTwist.cpp)
target_link_libraries(setVehicleTwist uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(setVehicleDisturbances interface_examples/setVehicleDisturbances.cpp)
target_link_libraries(setVehicleDisturbances uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(gotoAbsolutePose interface_examples/gotoAbsolutePose.cpp)
target_link_libraries(gotoAbsolutePose uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(gotoRelativePose interface_examples/gotoRelativePose.cpp)
target_link_libraries(gotoRelativePose uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(setVehicleActuators interface_examples/setVehicleActuators.cpp)
target_link_libraries(setVehicleActuators uwsim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(followMarker interface_examples/followMarker.cpp)
target_link_libraries(followMarker ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(spawnMarker interface_examples/spawnMarker.cpp)
target_link_libraries(spawnMarker ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(spawnMarker underwater_sensor_msgs_gencpp)


install(
  TARGETS
    uwsim
    uwsim_binary
    uwsim_plugins_simdev
    makeVehicleSurvey
    setJointPosition
    setJointVelocity
    setVehiclePosition
    setVehiclePose
    setVehicleVelocity
    setVehicleTwist
    setVehicleDisturbances
    gotoAbsolutePose
    gotoRelativePose
    setVehicleActuators
    followMarker
    spawnMarker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

##compìle xacros
file(GLOB xacro_files data/scenes/*.xml.xacro)

foreach(it ${xacro_files})
  # remove .xacro extension
  string(REGEX MATCH "(.*)[.]xacro$" unused ${it})
  set(output_filename ${CMAKE_MATCH_1})

  # create a rule to generate ${output_filename} from {it}
  xacro_add_xacro_file(${it} ${output_filename})

  list(APPEND world_files ${output_filename})
endforeach(it)

# add an abstract target to actually trigger the builds
add_custom_target(media_files ALL DEPENDS ${world_files})

install(DIRECTORY data/shaders
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
   PATTERN "*.xacro" EXCLUDE ##Exclude .xacro files
)

install(DIRECTORY src/
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   FILES_MATCHING PATTERN "uwsim" 
   PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
               GROUP_READ GROUP_EXECUTE
               WORLD_READ WORLD_EXECUTE
)

install(FILES package.xml simdev_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
