# Install script for directory: /home/lsa/ocean2/src/uwsim_osgworks/src/osgwQuery

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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "libosgworks")
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgwQuery.so.3.00.00"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgwQuery.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/lsa/ocean2/build_isolated/uwsim_osgworks/install/lib/libosgwQuery.so.3.00.00"
    "/home/lsa/ocean2/build_isolated/uwsim_osgworks/install/lib/libosgwQuery.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgwQuery.so.3.00.00"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgwQuery.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/lsa/ocean2/build_isolated/uwsim_osgworks/install/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "libosgworks-dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/uwsim_osgworks/osgwQuery" TYPE FILE FILES
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwQuery/QueryAPI.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwQuery/QueryBenchmarks.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwQuery/QueryComputation.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwQuery/QueryObject.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwQuery/QueryStats.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwQuery/QueryUtils.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwQuery/Export.h"
    )
endif()

