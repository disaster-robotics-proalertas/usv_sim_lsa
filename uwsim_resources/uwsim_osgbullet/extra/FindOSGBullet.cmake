# Locate osgBullet.
#
# This script defines:
#   OSGBULLET_FOUND, set to 1 if found
#   OSGBCOLLISION_LIBRARY
#   OSGBINTERACTION_LIBRARY
#   OSGBDYNAMICS_LIBRARY
#   OSGBINTERACTION_LIBRARY_debug
#   OSGBDYNAMICS_LIBRARY_debug
#   OSGBCOLLISION_LIBRARY_debug
#   OSGBULLET_LIBRARIES, all osgBullet libraries
#   OSGBULLET_INCLUDE_DIR
#
# This script will look in standard locations for installed osgBullet. However, if you
# install osgBullet into a non-standard location, you can use the OSGBULLET_ROOT
# variable (in environment or CMake) to specify the location.
#
# You can also use osgBullet out of a source tree by specifying OSGBULLET_SOURCE_DIR
# and OSGBULLET_BUILD_DIR (in environment or CMake).


SET( OSGBULLET_BUILD_DIR "" CACHE PATH "If using osgBullet out of a source tree, specify the build directory." )
SET( OSGBULLET_SOURCE_DIR "" CACHE PATH "If using osgBullet out of a source tree, specify the root of the source tree." )
SET( OSGBULLET_ROOT "" CACHE PATH "Specify non-standard osgBullet install directory. It is the parent of the include and lib dirs." )

MACRO( FIND_OSGBULLET_INCLUDE THIS_OSGBULLET_INCLUDE_DIR THIS_OSGBULLET_INCLUDE_FILE )
    UNSET( ${THIS_OSGBULLET_INCLUDE_DIR} )
    MARK_AS_ADVANCED( ${THIS_OSGBULLET_INCLUDE_DIR} )
    FIND_PATH( ${THIS_OSGBULLET_INCLUDE_DIR} ${THIS_OSGBULLET_INCLUDE_FILE}
        PATHS
            ${OSGBULLET_ROOT}
            $ENV{OSGBULLET_ROOT}
            ${OSGBULLET_SOURCE_DIR}
            $ENV{OSGBULLET_SOURCE_DIR}
            /usr/local
            /usr
            /sw/ # Fink
            /opt/local # DarwinPorts
            /opt/csw # Blastwave
            /opt
            "C:/Program Files/osgBullet"
            "C:/Program Files (x86)/osgBullet"
            ~/Library/Frameworks
            /Library/Frameworks
        PATH_SUFFIXES
            include
            .
    )
ENDMACRO( FIND_OSGBULLET_INCLUDE THIS_OSGBULLET_INCLUDE_DIR THIS_OSGBULLET_INCLUDE_FILE )

FIND_OSGBULLET_INCLUDE( OSGBULLET_INCLUDE_DIR osgbDynamics/MotionState.h )
# message( STATUS ${OSGBULLET_INCLUDE_DIR} )

MACRO( FIND_OSGBULLET_LIBRARY MYLIBRARY MYLIBRARYNAME )
    UNSET( ${MYLIBRARY} CACHE )
    UNSET( ${MYLIBRARY}_debug CACHE )
    MARK_AS_ADVANCED( ${MYLIBRARY} )
    MARK_AS_ADVANCED( ${MYLIBRARY}_debug )
    FIND_LIBRARY( ${MYLIBRARY}
        NAMES
            ${MYLIBRARYNAME}
        PATHS
            ${OSGBULLET_ROOT}
            $ENV{OSGBULLET_ROOT}
            ${OSGBULLET_BUILD_DIR}
            $ENV{OSGBULLET_BUILD_DIR}
            ~/Library/Frameworks
            /Library/Frameworks
            /usr/local
            /usr
            /sw
            /opt/local
            /opt/csw
            /opt
            "C:/Program Files/osgBullet"
            "C:/Program Files (x86)/osgBullet"
            /usr/freeware/lib64
        PATH_SUFFIXES
            lib
            bin/Release
            .
    )
    FIND_LIBRARY( ${MYLIBRARY}_debug
        NAMES
            ${MYLIBRARYNAME}d
        PATHS
            ${OSGBULLET_ROOT}
            $ENV{OSGBULLET_ROOT}
            ${OSGBULLET_BUILD_DIR}
            $ENV{OSGBULLET_BUILD_DIR}
            ~/Library/Frameworks
            /Library/Frameworks
            /usr/local
            /usr
            /sw
            /opt/local
            /opt/csw
            /opt
            "C:/Program Files/osgBullet"
            "C:/Program Files (x86)/osgBullet"
            /usr/freeware/lib64
        PATH_SUFFIXES
            lib
            bin/Debug
            .
    )
#    message( STATUS ${${MYLIBRARY}} ${${MYLIBRARY}_debug} )
#    message( STATUS ${MYLIBRARYNAME} )
    IF( ${MYLIBRARY} )
        SET( OSGBULLET_LIBRARIES ${OSGBULLET_LIBRARIES}
            "optimized" ${${MYLIBRARY}}
        )
    ENDIF( ${MYLIBRARY} )
    IF( ${MYLIBRARY}_debug )
        SET( OSGBULLET_LIBRARIES ${OSGBULLET_LIBRARIES}
            "debug" ${${MYLIBRARY}_debug}
        )
    ENDIF( ${MYLIBRARY}_debug )
ENDMACRO(FIND_OSGBULLET_LIBRARY LIBRARY LIBRARYNAME)

FIND_OSGBULLET_LIBRARY( OSGBINTERACTION_LIBRARY osgbInteraction )
FIND_OSGBULLET_LIBRARY( OSGBDYNAMICS_LIBRARY osgbDynamics )
FIND_OSGBULLET_LIBRARY( OSGBCOLLISION_LIBRARY osgbCollision )

SET( OSGBULLET_FOUND 0 )
IF( OSGBULLET_LIBRARIES AND OSGBULLET_INCLUDE_DIR )
    SET( OSGBULLET_FOUND 1 )
ENDIF( OSGBULLET_LIBRARIES AND OSGBULLET_INCLUDE_DIR )
