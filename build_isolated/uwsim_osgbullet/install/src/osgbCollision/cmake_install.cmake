# Install script for directory: /home/lsa/ocean2/src/uwsim_osgbullet/src/osgbCollision

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lsa/ocean2/install_isolated")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "libosgbbullet")
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgbCollision.so.3.00.00"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgbCollision.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/lsa/ocean2/build_isolated/uwsim_osgbullet/install/lib/libosgbCollision.so.3.00.00"
    "/home/lsa/ocean2/build_isolated/uwsim_osgbullet/install/lib/libosgbCollision.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgbCollision.so.3.00.00"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgbCollision.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/lsa/ocean2/install_isolated/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "libosgbbullet-dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/uwsim_osgbullet/osgbCollision" TYPE FILE FILES
    "/home/lsa/ocean2/src/uwsim_osgbullet/include/osgbCollision/BoundingCone.h"
    "/home/lsa/ocean2/src/uwsim_osgbullet/include/osgbCollision/BoundingCylinder.h"
    "/home/lsa/ocean2/src/uwsim_osgbullet/include/osgbCollision/Chart.h"
    "/home/lsa/ocean2/src/uwsim_osgbullet/include/osgbCollision/CollectVerticesVisitor.h"
    "/home/lsa/ocean2/src/uwsim_osgbullet/include/osgbCollision/CollisionShapes.h"
    "/home/lsa/ocean2/src/uwsim_osgbullet/include/osgbCollision/ComputeCylinderVisitor.h"
    "/home/lsa/ocean2/src/uwsim_osgbullet/include/osgbCollision/ComputeShapeVisitor.h"
    "/home/lsa/ocean2/src/uwsim_osgbullet/include/osgbCollision/ComputeTriMeshVisitor.h"
    "/home/lsa/ocean2/src/uwsim_osgbullet/include/osgbCollision/GLDebugDrawer.h"
    "/home/lsa/ocean2/src/uwsim_osgbullet/include/osgbCollision/RefBulletObject.h"
    "/home/lsa/ocean2/src/uwsim_osgbullet/include/osgbCollision/Utils.h"
    "/home/lsa/ocean2/src/uwsim_osgbullet/include/osgbCollision/Version.h"
    "/home/lsa/ocean2/src/uwsim_osgbullet/include/osgbCollision/VertexAggOp.h"
    "/home/lsa/ocean2/src/uwsim_osgbullet/include/osgbCollision/Export.h"
    )
endif()

