# Install script for directory: /home/lsa/ocean2/src/uwsim_osgworks/src/osgwTools

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
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgwTools.so.3.00.00"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgwTools.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/lsa/ocean2/build_isolated/uwsim_osgworks/install/lib/libosgwTools.so.3.00.00"
    "/home/lsa/ocean2/build_isolated/uwsim_osgworks/install/lib/libosgwTools.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgwTools.so.3.00.00"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosgwTools.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "libosgworks-dev")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/uwsim_osgworks/osgwTools" TYPE FILE FILES
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/Export.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/AbsoluteModelTransform.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/CallbackSupport.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/CameraConfigObject.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/Capabilities.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/CollapseLOD.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/CountsVisitor.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/DataLoader.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/DecimationTestModel.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/FBOUtils.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/FindNamedNode.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/ForceFlattenTransforms.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/GeometryModifier.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/GeometryOperation.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/InsertRemove.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/LODCreationNodeVisitor.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/MultiCameraProjectionMatrix.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/NodePathUtils.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/NodeUtils.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/Orientation.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/ParallelVisitor.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/PrimitiveSetConversion.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/Quat.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/ReadFile.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/ReducerOp.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/RefID.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/RemoveData.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/RemoveLOD.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/RemoveProgram.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/ScreenCapture.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/SerializerSupport.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/Shapes.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/SimplifierOp.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/StateSetUtils.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/StateTrackingNodeVisitor.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/Transform.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/TransformUtils.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/TransparencyUtils.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/Trianglizer.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/Uniqifier.h"
    "/home/lsa/ocean2/src/uwsim_osgworks/include/osgwTools/Version.h"
    )
endif()

