# Install script for directory: /home/lsa/ocean2/src/uwsim_bullet/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lsa/ocean2/devel_isolated/uwsim_bullet")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/btBulletCollisionCommon.h;/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/btBulletDynamicsCommon.h;/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/Bullet-C-Api.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet" TYPE FILE FILES
    "/home/lsa/ocean2/src/uwsim_bullet/src/btBulletCollisionCommon.h"
    "/home/lsa/ocean2/src/uwsim_bullet/src/btBulletDynamicsCommon.h"
    "/home/lsa/ocean2/src/uwsim_bullet/src/Bullet-C-Api.h"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/vectormath/vmInclude.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/vectormath" TYPE FILE FILES "/home/lsa/ocean2/src/uwsim_bullet/src/vectormath/vmInclude.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/vectormath/scalar/boolInVec.h;/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/vectormath/scalar/floatInVec.h;/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/vectormath/scalar/mat_aos.h;/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/vectormath/scalar/quat_aos.h;/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/vectormath/scalar/vec_aos.h;/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/vectormath/scalar/vectormath_aos.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/vectormath/scalar" TYPE FILE FILES
    "/home/lsa/ocean2/src/uwsim_bullet/src/vectormath/scalar/boolInVec.h"
    "/home/lsa/ocean2/src/uwsim_bullet/src/vectormath/scalar/floatInVec.h"
    "/home/lsa/ocean2/src/uwsim_bullet/src/vectormath/scalar/mat_aos.h"
    "/home/lsa/ocean2/src/uwsim_bullet/src/vectormath/scalar/quat_aos.h"
    "/home/lsa/ocean2/src/uwsim_bullet/src/vectormath/scalar/vec_aos.h"
    "/home/lsa/ocean2/src/uwsim_bullet/src/vectormath/scalar/vectormath_aos.h"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/vectormath/sse/boolInVec.h;/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/vectormath/sse/floatInVec.h;/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/vectormath/sse/mat_aos.h;/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/vectormath/sse/quat_aos.h;/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/vectormath/sse/vec_aos.h;/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/vectormath/sse/vecidx_aos.h;/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/vectormath/sse/vectormath_aos.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/lsa/ocean2/devel_isolated/uwsim_bullet/include/uwsim_bullet/vectormath/sse" TYPE FILE FILES
    "/home/lsa/ocean2/src/uwsim_bullet/src/vectormath/sse/boolInVec.h"
    "/home/lsa/ocean2/src/uwsim_bullet/src/vectormath/sse/floatInVec.h"
    "/home/lsa/ocean2/src/uwsim_bullet/src/vectormath/sse/mat_aos.h"
    "/home/lsa/ocean2/src/uwsim_bullet/src/vectormath/sse/quat_aos.h"
    "/home/lsa/ocean2/src/uwsim_bullet/src/vectormath/sse/vec_aos.h"
    "/home/lsa/ocean2/src/uwsim_bullet/src/vectormath/sse/vecidx_aos.h"
    "/home/lsa/ocean2/src/uwsim_bullet/src/vectormath/sse/vectormath_aos.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/lsa/ocean2/build_isolated/uwsim_bullet/devel/src/BulletSoftBody/cmake_install.cmake")
  include("/home/lsa/ocean2/build_isolated/uwsim_bullet/devel/src/BulletCollision/cmake_install.cmake")
  include("/home/lsa/ocean2/build_isolated/uwsim_bullet/devel/src/BulletDynamics/cmake_install.cmake")
  include("/home/lsa/ocean2/build_isolated/uwsim_bullet/devel/src/LinearMath/cmake_install.cmake")

endif()

