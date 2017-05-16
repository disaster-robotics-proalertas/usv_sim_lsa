# - Find a osgBullet installation or build tree.
# The following variables are set if osgBullet is found.  If osgBullet is not
# found, osgBullet_FOUND is set to false.
#  osgBullet_FOUND         - Set to true when osgBullet is found.
#  osgBullet_USE_FILE      - CMake file to use osgBullet.
#  osgBullet_MAJOR_VERSION - The osgBullet major version number.
#  osgBullet_MINOR_VERSION - The osgBullet minor version number 
#                       (odd non-release).
#  osgBullet_BUILD_VERSION - The osgBullet patch level 
#                       (meaningless for odd minor).
#  osgBullet_INCLUDE_DIRS  - Include directories for osgBullet
#  osgBullet_LIBRARY_DIRS  - Link directories for osgBullet libraries

# The following cache entries must be set by the user to locate osgBullet:
#  osgBullet_DIR  - The directory containing osgBulletConfig.cmake.  
#             This is either the root of the build tree,
#             or the lib directory.  This is the 
#             only cache entry.


# Assume not found.
SET(osgBullet_FOUND 0)

# Construct consitent error messages for use below.
SET(osgBullet_DIR_DESCRIPTION "directory containing osgBulletConfig.cmake.  This is either the root of the build tree, or PREFIX/lib for an installation.")
SET(osgBullet_DIR_MESSAGE "osgBullet not found.  Set the osgBullet_DIR cmake cache entry to the ${osgBullet_DIR_DESCRIPTION}")

# Use the Config mode of the find_package() command to find osgBulletConfig.
# If this succeeds (possibly because osgBullet_DIR is already set), the
# command will have already loaded osgBulletConfig.cmake and set osgBullet_FOUND.
IF(NOT osgBullet_FOUND)
  FIND_PACKAGE(osgBullet QUIET NO_MODULE)
ENDIF(NOT osgBullet_FOUND)

#-----------------------------------------------------------------------------
IF(NOT osgBullet_FOUND)
  # osgBullet not found, explain to the user how to specify its location.
  IF(osgBullet_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR ${osgBullet_DIR_MESSAGE})
  ELSE(osgBullet_FIND_REQUIRED)
    IF(NOT osgBullet_FIND_QUIETLY)
      MESSAGE(STATUS ${osgBullet_DIR_MESSAGE})
    ENDIF(NOT osgBullet_FIND_QUIETLY)
  ENDIF(osgBullet_FIND_REQUIRED)
ENDIF(NOT osgBullet_FOUND)
