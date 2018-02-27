# The name of our project is "osgOcean". CMakeLists files in this project can
# refer to the root source directory of the project as ${osgOcean_SOURCE_DIR}
# to the root binary directory of the project as ${osgOcean_BINARY_DIR}.
cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project (uwsim_osgocean)

SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

# Work around for building an xcode project. It seems XCode projects that
# built with CMake are not compatible with relative paths.
IF(NOT APPLE)
  SET( CMAKE_USE_RELATIVE_PATHS true )
ENDIF()

SET(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "add a postfix, usually d")
SET(CMAKE_RELEASE_POSTFIX "" CACHE STRING "add a postfix, usually empty")
SET(CMAKE_RELWITHDEBINFO_POSTFIX "" CACHE STRING "add a postfix, usually rd")
SET(CMAKE_MINSIZEREL_POSTFIX "s" CACHE STRING "add a postfix, usually s")

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules;{CMAKE_MODULE_PATH}")

find_package (osg REQUIRED)
include_directories (${OSG_INCLUDE_DIR})

find_package (osgDB REQUIRED)
include_directories (${OSGDB_INCLUDE_DIR})

find_package (osgGA REQUIRED)
include_directories (${OSGGA_INCLUDE_DIR})

find_package (osgViewer REQUIRED)
include_directories (${OSGVIEWER_INCLUDE_DIR})

find_package (osgUtil REQUIRED)
include_directories (${OSGUTIL_INCLUDE_DIR})

find_package (osgText REQUIRED)
include_directories (${OSGTEXT_INCLUDE_DIR})

find_package (OpenThreads REQUIRED)
include_directories(${OPENTHREADS_INCLUDE_DIR})

find_package (osgShadow REQUIRED)
include_directories (${OSGSHADOW_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

IF (NOT WIN32)
  IF (NOT OSG_LIBRARY_DEBUG)
    SET(OSG_LIBRARY_DEBUG ${OSG_LIBRARY})
  ENDIF()
  IF (NOT OSGDB_LIBRARY_DEBUG)
    SET(OSGDB_LIBRARY_DEBUG ${OSGDB_LIBRARY})
  ENDIF()
  IF (NOT OSGGA_LIBRARY_DEBUG)
    SET(OSGGA_LIBRARY_DEBUG ${OSGGA_LIBRARY})
  ENDIF()
  IF (NOT OSGVIEWER_LIBRARY_DEBUG)
    SET(OSGVIEWER_LIBRARY_DEBUG ${OSGVIEWER_LIBRARY})
  ENDIF()
  IF (NOT OSGUTIL_LIBRARY_DEBUG)
    SET(OSGUTIL_LIBRARY_DEBUG ${OSGUTIL_LIBRARY})
  ENDIF()
  IF (NOT OSGTEXT_LIBRARY_DEBUG)
    SET(OSGTEXT_LIBRARY_DEBUG ${OSGTEXT_LIBRARY})
  ENDIF()
  IF (NOT OPENTHREADS_LIBRARY_DEBUG)
    SET(OPENTHREADS_LIBRARY_DEBUG ${OPENTHREADS_LIBRARY})
  ENDIF()
ENDIF()

find_package (OpenGL REQUIRED)
include_directories (${OPENGL_INCLUDE_DIR})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

OPTION(USE_FFTW3 "Use FFTW3 (double-precision) (GPL) as FFT library." OFF)
OPTION(USE_FFTW3F "Use FFTW3 (single-precision) (GPL) as FFT library." ON)
OPTION(USE_FFTSS "Use FFTSS (LGPL) as FFT library." OFF)

# How do I enforce that only one of USE_FFTW3, USE_FFTW3F or USE_FFTSS should
# be selected at one time? Is there the concept of a radio-button or
# single-selection list in CMake? Right now it will just use the first one
# that is checked in the order below.

IF(USE_FFTSS)
  find_package (fftss REQUIRED)
  MESSAGE(STATUS "Using FFTSS (LGPL) as FFT library.")

  ADD_DEFINITIONS(-DUSE_FFTSS)
  SET( FFT_INCLUDE_DIR ${FFTSS_INCLUDE_DIR} )
  SET( FFT_LIBRARY ${FFTSS_LIBRARY} )

  SET(USE_FFTW3 FALSE)
  SET(USE_FFTW3F FALSE)
ELSEIF(USE_FFTW3F)
  find_package (fftw3f-3 REQUIRED)
  MESSAGE(STATUS "Using FFTW3F (single-precision) (GPL) as FFT library.")

  ADD_DEFINITIONS(-DUSE_FFTW3F)
  SET( FFT_INCLUDE_DIR ${FFTW3F-3_INCLUDE_DIR} )
  SET( FFT_LIBRARY ${FFTW3F-3_LIBRARY} )

  SET(USE_FFTW3 FALSE)
  SET(USE_FFTSS FALSE)
ELSEIF(USE_FFTW3)
  find_package (fftw3-3 REQUIRED)
  MESSAGE(STATUS "Using FFTW3 (double-precision) (GPL) as FFT library.")

  ADD_DEFINITIONS(-DUSE_FFTW3)
  SET( FFT_INCLUDE_DIR ${FFTW3-3_INCLUDE_DIR} )
  SET( FFT_LIBRARY ${FFTW3-3_LIBRARY} )

  SET(USE_FFTW3F FALSE)
  SET(USE_FFTSS FALSE)
ELSE()
  # Error if none of the three is selected.
  MESSAGE("No FFT library selected, you will not be able to generate ocean surfaces.")
ENDIF()


IF (WIN32)
  # This option is to enable the /DYNAMICBASE switch
  # It is used to workaround a bug in Windows 7 when linking in release, which results in corrupt
  # binaries. See this page for details: http://www.wintellect.com/CS/blogs/jrobbins/archive/2009/01/24/the-case-of-the-corrupt-pe-binaries.aspx
  OPTION(WIN32_USE_DYNAMICBASE "Set to ON to build osgOcean with the /DYNAMICBASE option to work around a bug when linking release executables on Windows 7." OFF)
  MARK_AS_ADVANCED(WIN32_USE_DYNAMICBASE)
  IF(WIN32_USE_DYNAMICBASE)
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /DYNAMICBASE")
  ENDIF(WIN32_USE_DYNAMICBASE)
ENDIF()


include( CMakePackageConfigHelpers )
# Create the osgWorksConfig.cmake and osgWorksConfigVersion files
file(RELATIVE_PATH CONF_REL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/lib"
   "${CMAKE_INSTALL_PREFIX}/include")

configure_package_config_file( 
    osgOceanConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/lib/${CMAKE_PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION lib
    PATH_VARS CONF_REL_INCLUDE_DIR )

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/lib/${CMAKE_PROJECT_NAME}Config.cmake 
        DESTINATION share/${CMAKE_PROJECT_NAME} )


# Install catkin package.xml
install(FILES package.xml DESTINATION share/uwsim_osgocean)

add_subdirectory (src)


