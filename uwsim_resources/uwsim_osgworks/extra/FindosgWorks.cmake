# - Find a osgWorks installation or build tree.
# The following variables are set if osgWorks is found.  If osgWorks is not
# found, osgWorks_FOUND is set to false.
#  osgWorks_FOUND         - Set to true when osgWorks is found.
#  osgWorks_USE_FILE      - CMake file to use osgWorks.
#  osgWorks_MAJOR_VERSION - The osgWorks major version number.
#  osgWorks_MINOR_VERSION - The osgWorks minor version number 
#                       (odd non-release).
#  osgWorks_BUILD_VERSION - The osgWorks patch level 
#                       (meaningless for odd minor).
#  osgWorks_INCLUDE_DIRS  - Include directories for osgWorks
#  osgWorks_LIBRARY_DIRS  - Link directories for osgWorks libraries

# The following cache entries must be set by the user to locate osgWorks:
#  osgWorks_DIR  - The directory containing osgWorksConfig.cmake.  
#             This is either the root of the build tree,
#             or the lib directory.  This is the 
#             only cache entry.


# Assume not found.
SET(osgWorks_FOUND 0)

# Construct consitent error messages for use below.
SET(osgWorks_DIR_DESCRIPTION "directory containing osgWorksConfig.cmake.  This is either the root of the build tree, or PREFIX/lib for an installation.")
SET(osgWorks_DIR_MESSAGE "osgWorks not found.  Set the osgWorks_DIR cmake cache entry to the ${osgWorks_DIR_DESCRIPTION}")

# Use the Config mode of the find_package() command to find osgWorksConfig.
# If this succeeds (possibly because osgWorks_DIR is already set), the
# command will have already loaded osgWorksConfig.cmake and set osgWorks_FOUND.
IF(NOT osgWorks_FOUND)
  FIND_PACKAGE(osgWorks QUIET NO_MODULE)
ENDIF(NOT osgWorks_FOUND)

#-----------------------------------------------------------------------------
IF(NOT osgWorks_FOUND)
  # osgWorks not found, explain to the user how to specify its location.
  IF(osgWorks_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR ${osgWorks_DIR_MESSAGE})
  ELSE(osgWorks_FIND_REQUIRED)
    IF(NOT osgWorks_FIND_QUIETLY)
      MESSAGE(STATUS ${osgWorks_DIR_MESSAGE})
    ENDIF(NOT osgWorks_FIND_QUIETLY)
  ENDIF(osgWorks_FIND_REQUIRED)
ENDIF(NOT osgWorks_FOUND)
