# sourced from https://wiki-flowvr.imag.fr/browser/trunk/flowvr-suite/flowvr-vrpn/cmake/FindVRPN.cmake
# No license is listed for FindVRPN.Cmake
# - Try to find VRPN
# Once done this will define
#
#  VRPN_FOUND - system has VRPN and  Quat
#  VRPN_INCLUDES - the VRPN include directory
#  VRPN_LIBRARY - Link these to use VRPN
#  QUAT_INCLUDES - the QUAT include directory
#  QUAT_LIBRARY - Link these to use QUAT

FIND_LIBRARY (VRPN_LIBRARY NAMES vrpn
    PATHS 
    ENV LD_LIBRARY_PATH
    ENV LIBRARY_PATH
    /usr/lib64
    /usr/lib
    /usr/local/lib64
    /usr/local/lib
    /opt/local/lib
    )
FIND_PATH (VRPN_INCLUDES vrpn_Keyboard.h
    PATHS
    ENV CPATH
    /usr/include
    /usr/local/include
    /opt/local/include
    PATH_SUFFIXES vrpn
    )
FIND_LIBRARY (QUAT_LIBRARY
    NAMES quat
    PATHS
    ENV LD_LIBRARY_PATH
    ENV LIBRARY_PATH
    /usr/lib64
    /usr/lib
    /usr/local/lib64
    /usr/local/lib
    /opt/local/lib
    )
FIND_PATH (QUAT_INCLUDES quat.h
    PATHS
    ENV CPATH
    /usr/include
    /usr/local/include
    /opt/local/include
    PATH_SUFFIXES quat
    )

IF(VRPN_INCLUDES AND VRPN_LIBRARY AND QUAT_INCLUDES AND QUAT_LIBRARY)
    SET(VRPN_FOUND TRUE)
ENDIF(VRPN_INCLUDES AND VRPN_LIBRARY AND QUAT_INCLUDES AND QUAT_LIBRARY)

IF(VRPN_FOUND)
  IF(NOT VRPN_FIND_QUIETLY)
    MESSAGE(STATUS "VRPN and QUAT Found: ${VRPN_LIBRARY}  ${QUAT_LIBRARY}")
  ENDIF(NOT VRPN_FIND_QUIETLY)
ELSE(VRPN_FOUND)
  IF(VRPN_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Could not find VRPN and/or QUAT")
  ENDIF(VRPN_FIND_REQUIRED)
ENDIF(VRPN_FOUND)

