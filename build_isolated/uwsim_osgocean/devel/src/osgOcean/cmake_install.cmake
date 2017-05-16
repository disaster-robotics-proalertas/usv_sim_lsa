# Install script for directory: /home/lsa/ocean2/src/uwsim_osgocean/src/osgOcean

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lsa/ocean2/devel_isolated/uwsim_osgocean")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}/home/lsa/ocean2/devel_isolated/uwsim_osgocean/lib/libosgOcean.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/lsa/ocean2/devel_isolated/uwsim_osgocean/lib/libosgOcean.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/lsa/ocean2/devel_isolated/uwsim_osgocean/lib/libosgOcean.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lsa/ocean2/devel_isolated/uwsim_osgocean/lib/libosgOcean.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lsa/ocean2/devel_isolated/uwsim_osgocean/lib" TYPE SHARED_LIBRARY FILES "/home/lsa/ocean2/build_isolated/uwsim_osgocean/devel/lib/libosgOcean.so")
  if(EXISTS "$ENV{DESTDIR}/home/lsa/ocean2/devel_isolated/uwsim_osgocean/lib/libosgOcean.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/lsa/ocean2/devel_isolated/uwsim_osgocean/lib/libosgOcean.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/lsa/ocean2/devel_isolated/uwsim_osgocean/lib/libosgOcean.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/Cylinder;/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/DistortionSurface;/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/FFTOceanTechnique;/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/FFTOceanSurface;/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/FFTOceanSurfaceVBO;/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/FFTSimulation;/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/GodRays;/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/GodRayBlendSurface;/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/MipmapGeometry;/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/MipmapGeometryVBO;/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/OceanScene;/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/OceanTechnique;/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/OceanTile;/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/RandUtils;/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/ScreenAlignedQuad;/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/ShaderManager;/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/SiltEffect;/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/WaterTrochoids;/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/Export;/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean/Version")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lsa/ocean2/devel_isolated/uwsim_osgocean/include/uwsim_osgocean/osgOcean" TYPE FILE FILES
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/Cylinder"
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/DistortionSurface"
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/FFTOceanTechnique"
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/FFTOceanSurface"
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/FFTOceanSurfaceVBO"
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/FFTSimulation"
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/GodRays"
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/GodRayBlendSurface"
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/MipmapGeometry"
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/MipmapGeometryVBO"
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/OceanScene"
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/OceanTechnique"
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/OceanTile"
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/RandUtils"
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/ScreenAlignedQuad"
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/ShaderManager"
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/SiltEffect"
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/WaterTrochoids"
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/Export"
    "/home/lsa/ocean2/src/uwsim_osgocean/include/osgOcean/Version"
    )
endif()

