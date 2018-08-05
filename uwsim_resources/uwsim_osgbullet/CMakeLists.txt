# # # # # #  project setup  # # # # # #

cmake_minimum_required( VERSION 2.8.7 )
project( uwsim_osgbullet )

# Please keep in sync with Version.h
set( OSGBULLET_VERSION 3.00.00 )
set( OSGBCOLLISION_MAJOR_VERSION 3 )
set( OSGBCOLLISION_MINOR_VERSION 0 )
set( OSGBCOLLISION_SUB_VERSION 0 )

set( CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}" )

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib )
if( WIN32 )
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} )
else()
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY} )
endif()

#
# Setup the parrallel build support for windows
#
IF( WIN32 AND MSVC )
    OPTION( OSGBULLET_WIN32_USE_MP "Build with multiple processes (/MP option; VS 2005 and above)." ON )
    MARK_AS_ADVANCED( OSGBULLET_WIN32_USE_MP )
    IF( OSGBULLET_WIN32_USE_MP )
        SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP" )
    ENDIF( OSGBULLET_WIN32_USE_MP )
ENDIF( WIN32 AND MSVC )

#
# Setup the the lib directory on a per platform basis
#
if(CMAKE_COMPILER_IS_GNUCXX)
    include(GNUInstallDirs)
endif(CMAKE_COMPILER_IS_GNUCXX)

if(NOT DEFINED CMAKE_INSTALL_LIBDIR)
    set(CMAKE_INSTALL_LIBDIR lib)
endif(NOT DEFINED CMAKE_INSTALL_LIBDIR)


IF(NOT ANDROID)
IF(APPLE)

    #build for iOS
    OPTION(OSG_BUILD_PLATFORM_IPHONE "Enable IPhoneSDK Device support" OFF)
    OPTION(OSG_BUILD_PLATFORM_IPHONE_SIMULATOR "Enable IPhoneSDK Simulator support" OFF)

    IF(OSG_BUILD_PLATFORM_IPHONE OR OSG_BUILD_PLATFORM_IPHONE_SIMULATOR)

        #you need to manually set the default sdk version here
        SET (IPHONE_SDKVER "6.0")

        #the below is taken from ogre, it states the gcc stuff needs to happen before PROJECT() is called. I've no clue if we even need it
        # Force gcc <= 4.2 on iPhone
        include(CMakeForceCompiler)
        CMAKE_FORCE_C_COMPILER(llvm-gcc-4.2 GNU)
        CMAKE_FORCE_CXX_COMPILER(llvm-gcc-4.2 GNU)
        SET(GCC_THUMB_SUPPORT NO)

        #set either the device sdk or the simulator sdk. Can't find away to separate these in the same project
        IF(OSG_BUILD_PLATFORM_IPHONE)
            SET (IPHONE_DEVROOT "/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Developer")
            SET (IPHONE_SDKROOT "${IPHONE_DEVROOT}/SDKs/iPhoneOS${IPHONE_SDKVER}.sdk")
        ELSE()
            SET (IPHONE_DEVROOT "/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneSimulator.platform/Developer")
            SET (IPHONE_SDKROOT "${IPHONE_DEVROOT}/SDKs/iPhoneSimulator${IPHONE_SDKVER}.sdk")
        ENDIF()
    ENDIF ()

    OPTION(OSG_COMPILE_FRAMEWORKS "compile frameworks instead of dylibs (experimental)" OFF)
    SET(OSG_COMPILE_FRAMEWORKS_INSTALL_NAME_DIR "@executable_path/../Frameworks" CACHE STRING "install name dir for compiled frameworks")
ENDIF()
ENDIF()

IF(APPLE)

        #Here we check if the user specified IPhone SDK
    IF(OSG_BUILD_PLATFORM_IPHONE OR OSG_BUILD_PLATFORM_IPHONE_SIMULATOR)

        #set iphone arch and flags taken from http://sites.google.com/site/michaelsafyan/coding/resources/how-to-guides/cross-compile-for-the-iphone/how-to-cross-compile-for-the-iphone-using-cmake
        IF(OSG_BUILD_PLATFORM_IPHONE)
            SET(CMAKE_OSX_ARCHITECTURES "armv7" CACHE STRING "Build architectures for iOS" FORCE)
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -miphoneos-version-min=5.1 -mno-thumb -arch armv7 -pipe -no-cpp-precomp" CACHE STRING "Flags used by the compiler during all build types." FORCE)
        ELSE()
            #simulator uses i386 architectures
            SET(CMAKE_OSX_ARCHITECTURES "i386" CACHE STRING "Build architectures for iOS Simulator" FORCE)
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mno-thumb -arch i386 -pipe -no-cpp-precomp" CACHE STRING "Flags used by the compiler during all build types." FORCE)
        ENDIF()

        #here we set the specific iphone sdk version. We can only set either device or simulator sdk. So if you want both you currently have to have two seperate projects
        SET(CMAKE_OSX_SYSROOT "${IPHONE_SDKROOT}" CACHE STRING "System root for iOS" FORCE)

        #hack, force link to opengles
        set(CMAKE_EXE_LINKER_FLAGS "-framework Foundation -framework OpenGLES")

        #use the IPhone windowing system
        SET(OSG_WINDOWING_SYSTEM "IOS" CACHE STRING "Forced IPhone windowing system on iOS"  FORCE)
        SET(OSG_DEFAULT_IMAGE_PLUGIN_FOR_OSX "imageio" CACHE STRING "Forced imageio default image plugin for iOS" FORCE)

        #I think this or similar will be required for IPhone apps
        OPTION(OSG_BUILD_APPLICATION_BUNDLES "Enable the building of applications and examples as OSX Bundles" ON)
    ENDIF()
ENDIF()

INCLUDE( CMakeMacros )

include( FindOSGHelper )
#include( FindBulletHelper )


FIND_PACKAGE( uwsim_bullet REQUIRED )
FIND_PACKAGE( uwsim_osgworks REQUIRED )

# OSGBULLET_USE_DOUBLE_PRECISION gets set to TRUE or FALSE in FindBulletHelper.cmake.
# If TRUE, add the BT double precision definition globally.
if( OSGBULLET_USE_DOUBLE_PRECISION )
    add_definitions( -DBT_USE_DOUBLE_PRECISION )
endif()


option( OSGBULLET_P5_SUPPORT "Enable to include support for the P5 glove and build the related test code" OFF )
if( OSGBULLET_P5_SUPPORT )
    add_definitions( -DUSE_P5 )
    set( OSGBULLET_P5_INCLUDE_DIR "" CACHE PATH "" )
    set( OSGBULLET_P5_LIBRARY "" CACHE FILEPATH "" )
else()
    unset( OSGBULLET_P5_INCLUDE_DIR CACHE )
    unset( OSGBULLET_P5_LIBRARY CACHE )
endif()


INCLUDE_DIRECTORIES( 
    ${PROJECT_SOURCE_DIR}/include
    ${OSG_INCLUDE_DIRS}
    ${OSGWORKS_INCLUDE_DIR}
    ${BULLET_INCLUDE_DIRS}
    ${BULLET_EXTRAS_INCLUDE_DIR}
    ${OSGBULLET_P5_INCLUDE_DIR}
)

message(STATUS "CMAKE_PREFIX_PATH= $ENV{CMAKE_PREFIX_PATH}")
message(STATUS "LD_LIBRARY_PATH= $ENV{LD_LIBRARY_PATH}")
message(STATUS "CMAKE_MODULE_PATH= ${CMAKE_MODULE_PATH}")
message(STATUS "OSGWORKS_INCLUDE_DIR = ${OSGWORKS_INCLUDE_DIR}")
message(STATUS "BULLET_INCLUDE_DIRS= ${BULLET_INCLUDE_DIRS}")

# Support for collision-only (no dynamics)
if( BULLET_DYNAMICS_FOUND )
    add_definitions( -DUSE_BULLET_DYNAMICS )
endif()

option( BUILD_SHARED_LIBS "Set to ON to build osgBullet for dynamic linking. Use OFF for static." ON )
if( BUILD_SHARED_LIBS )
    add_definitions( -DOSGBULLET_SHARED )
else()
    add_definitions( -DOSGBULLET_STATIC )
endif()



OPTION( OSGBULLET_BUILD_APPLICATIONS "Enable to build applications" OFF )
IF( OSGBULLET_BUILD_APPLICATIONS )
    ADD_SUBDIRECTORY( applications )
ENDIF( OSGBULLET_BUILD_APPLICATIONS )

OPTION( OSGBULLET_BUILD_EXAMPLES "Enable to build example code" OFF )
IF( OSGBULLET_BUILD_EXAMPLES )
    ADD_SUBDIRECTORY( examples )
ENDIF( OSGBULLET_BUILD_EXAMPLES )

option( OSGBULLET_BUILD_TESTS "Enable to build test code" OFF )
if( OSGBULLET_BUILD_TESTS )
    enable_testing()
    add_subdirectory( tests )
endif()

option( OSGBULLET_INSTALL_DATA "Enable to add the data directory to the install target" OFF )
if( OSGBULLET_INSTALL_DATA )
    add_subdirectory( data )
endif()


ADD_SUBDIRECTORY( src/osgbCollision )
ADD_SUBDIRECTORY( src/osgbDynamics )
ADD_SUBDIRECTORY( src/osgbInteraction )
ADD_SUBDIRECTORY( src/plugins )

#
# Doxygen
find_package( Doxygen )
if( DOXYGEN_FOUND )
    set( HAVE_DOT "NO" )
    if( DOXYGEN_DOT_PATH )
        set( HAVE_DOT "YES" )
    endif()
    
    configure_file( ${PROJECT_SOURCE_DIR}/doc/doxyfile.cmake
        ${PROJECT_BINARY_DIR}/doc/doxyfile
    )
    add_custom_target( Documentation ${DOXYGEN_EXECUTABLE}
        ${PROJECT_BINARY_DIR}/doc/doxyfile
    )
endif()

# On windows provide the user with the pdb files 
# for debugging if they are present
if(MSVC)
    install(
        DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/\${CMAKE_INSTALL_CONFIG_NAME}/
        DESTINATION lib
        USE_SOURCE_PERMISSIONS
        COMPONENT libosgbbullet-dev
        FILES_MATCHING PATTERN "*.pdb"
    )
endif(MSVC)


include( CMakePackageConfigHelpers )
# Create the osgWorksConfig.cmake and osgWorksConfigVersion files
file(RELATIVE_PATH CONF_REL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/lib"
   "${CMAKE_INSTALL_PREFIX}/include")

configure_package_config_file( 
    osgBulletConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/lib/${CMAKE_PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION lib
    PATH_VARS CONF_REL_INCLUDE_DIR )

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/lib/${CMAKE_PROJECT_NAME}Config.cmake 
        DESTINATION share/${CMAKE_PROJECT_NAME} )

# Install catkin package.xml
install(FILES package.xml DESTINATION share/${CMAKE_PROJECT_NAME})

