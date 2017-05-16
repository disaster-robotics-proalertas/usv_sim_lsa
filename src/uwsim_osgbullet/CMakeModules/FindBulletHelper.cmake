
set( _bulletComponents
    BULLET_DYNAMICS BULLET_COLLISION BULLET_MATH BULLET_SOFTBODY
)

set( _defaultBulletLocations
    "C:/Program Files/BULLET_PHYSICS"
)

# Macro to force the stock FindBullet.cmake script to
# search again for Bullet.
macro( unFindBullet )
#    message( STATUS "In unFindBullet" )

    unset( BULLET_LIBRARIES CACHE )
    unset( BULLET_INCLUDE_DIR CACHE )
    unset( BULLET_INCLUDE_DIRS CACHE )
    foreach( currentLibIn ${_bulletComponents} )
        string( TOUPPER ${currentLibIn} currentLib )
        unset( "${currentLib}_INCLUDE_DIR" CACHE )
        unset( "${currentLib}_LIBRARIES" CACHE )
        unset( "${currentLib}_LIBRARY" CACHE )
        unset( "${currentLib}_LIBRARY_DEBUG" CACHE )
    endforeach() 
endmacro( unFindBullet )


# What type of Bullet should we look for? This is a combo box
# supporting three types of Bullet installations:
#  * Default installation (in which the stock FindBullet.cmake script does all the work)
#  * Alternate Install Location (user must set the BulletInstallLocation variable)
#  * Source And Build Tree (user must supply both the BulletSourceRoot and BulletBuildRoot variables)
set( BulletInstallType "Default Installation" CACHE STRING "Type of Bullet install: 'Default Installation', 'Alternate Install Location', or 'Source And Build Tree'." )
set_property( CACHE BulletInstallType PROPERTY STRINGS "Default Installation" "Alternate Install Location" "Source And Build Tree" )

# We need to detect when the user changes the Bullet install type
# or any of the related directory variables, so that we'll know
# to call unFindBullet() and force the stock Bullet script to search
# again. To do this, we save the last set value of these variables
# in the CMake cache as internal (hidden) variables.
if( NOT DEFINED _lastBulletInstallType )
    set( _lastBulletInstallType "empty" CACHE INTERNAL "" )
endif()
if( NOT DEFINED _lastBulletInstallLocation )
    set( _lastBulletInstallLocation "empty" CACHE INTERNAL "" )
endif()
if( NOT DEFINED _lastBulletSourceRoot )
    set( _lastBulletSourceRoot "empty" CACHE INTERNAL "" )
endif()
if( NOT DEFINED _lastBulletBuildRoot )
    set( _lastBulletBuildRoot "empty" CACHE INTERNAL "" )
endif()

if( NOT DEFINED BulletInstallLocation )
    set( BulletInstallLocation "Please specify" )
endif()
if( NOT DEFINED BulletSourceRoot )
    set( BulletSourceRoot "Please specify" )
endif()
if( NOT DEFINED BulletBuildRoot )
    set( BulletBuildRoot "Please specify" )
endif()


# If the user has changed the Bullet install type combo box
# (or it's a clean cache), then set or unset the our related
# Bullet directory search variables.
if( NOT ( ${BulletInstallType} STREQUAL ${_lastBulletInstallType} ) )
#    message( STATUS "NOT ( ${BulletInstallType} STREQUAL ${_lastBulletInstallType} )" )

    if( BulletInstallType STREQUAL "Default Installation" )
        # Remove our helper variables and tell the stock script to search again.
        unset( BulletInstallLocation CACHE )
        unset( BulletSourceRoot CACHE )
        unset( BulletBuildRoot CACHE )
    elseif( BulletInstallType STREQUAL "Alternate Install Location" )
        # Enable just the BulletInstallLocation helper variable.
        set( BulletInstallLocation "Please specify" CACHE PATH "Root directory where Bullet is installed" )
        unset( BulletSourceRoot CACHE )
        unset( BulletBuildRoot CACHE )
    elseif( BulletInstallType STREQUAL "Source And Build Tree" )
        # Enable the BulletSourceRoot and BulletBuildRoot helper variables.
        unset( BulletInstallLocation CACHE )
        set( BulletSourceRoot "Please specify" CACHE PATH "Root directory of Bullet source tree" )
        set( BulletBuildRoot "Please specify" CACHE PATH "Root directory of Bullet build tree" )
    endif()
endif()

# Library suffix needed on Windows to find libraries in build tree.
set( _bulletLibraryPathSuffix "" )
set( _bulletLibraryPathSuffixDebug "" )

# Suffix needed to find headers in a source tree.
set( _bulletSourceSuffix "" )

# Look for conditions that require us to find Bullet again.
set( _needToFindBullet FALSE )
if( BulletInstallType STREQUAL "Default Installation" )
    if( NOT ( ${BulletInstallType} STREQUAL ${_lastBulletInstallType} ) )
#        message( STATUS "Need to find: case A" )
        set( _needToFindBullet TRUE )
    endif()
elseif( BulletInstallType STREQUAL "Alternate Install Location" )
    if( NOT ( "${BulletInstallLocation}" STREQUAL "${_lastBulletInstallLocation}" ) )
#        message( STATUS "Need to find: case B" )
        set( _needToFindBullet TRUE )
    endif()
elseif( BulletInstallType STREQUAL "Source And Build Tree" )
    if( ( NOT ( "${BulletSourceRoot}" STREQUAL "${_lastBulletSourceRoot}" ) ) OR
        ( NOT ( "${BulletBuildRoot}" STREQUAL "${_lastBulletBuildRoot}" ) ) )
#        message( STATUS "Need to find: cade C" )
        set( _needToFindBullet TRUE )
    endif()
    set( _bulletSourceSuffix "/src" )
    if( WIN32 )
        set( _bulletLibraryPathSuffix "/lib/Release" )
        set( _bulletLibraryPathSuffixDebug "/lib/Debug" )
    endif()
endif()
if( _needToFindBullet )
    unFindBullet()
    set( _lastBulletInstallType ${BulletInstallType} CACHE INTERNAL "" FORCE )
    set( _lastBulletInstallLocation ${BulletInstallLocation} CACHE INTERNAL "" FORCE )
    set( _lastBulletSourceRoot ${BulletSourceRoot} CACHE INTERNAL "" FORCE )
    set( _lastBulletBuildRoot ${BulletBuildRoot} CACHE INTERNAL "" FORCE )
endif()



# Save internal variables for later restoration
set( CMAKE_PREFIX_PATH_SAVE ${CMAKE_PREFIX_PATH} )
set( CMAKE_LIBRARY_PATH_SAVE ${CMAKE_LIBRARY_PATH} )

set( CMAKE_PREFIX_PATH
    ${BulletInstallLocation}
    ${BulletSourceRoot}
    ${BulletSourceRoot}${_bulletSourceSuffix}
    ${BulletBuildRoot}
)
set( CMAKE_LIBRARY_PATH
    ${BulletInstallLocation}
    ${BulletBuildRoot}
    ${BulletBuildRoot}${_bulletLibraryPathSuffix}
    ${BulletBuildRoot}${_bulletLibraryPathSuffixDebug}
)
if( BulletInstallType STREQUAL "Default Installation" )
    list( APPEND CMAKE_PREFIX_PATH
        ${_defaultBulletLocations} )
    list( APPEND CMAKE_LIBRARY_PATH
        ${_defaultBulletLocations} )
endif()


find_package( Bullet )


# Restore internal variables
set( CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH_SAVE} )
set( CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH_SAVE} )

# Mark Bullet variables as advanced
mark_as_advanced( BULLET_INCLUDE_DIR )


# If we had to look for Bullet, *and* we found it,
# then let's see whether Bullet was built using
# double precision or not...
# By default, bullet is NOT built with double precision.
#
set( OSGBULLET_USE_DOUBLE_PRECISION FALSE CACHE BOOL "Select to force compiling with -DBT_USE_DOUBLE_PRECISION." )
if( _needToFindBullet AND BULLET_FOUND )
    message( STATUS "Testing Bullet for use of double precision..." )
    set( _result )
    set( _buildOut )
    
    # Configure for the correct build type to allow successful VS 2010 links
    # if Bullet was built release-only.
    if( BULLET_DYNAMICS_LIBRARY )
        set( CMAKE_TRY_COMPILE_CONFIGURATION Release )
    else()
        set( CMAKE_TRY_COMPILE_CONFIGURATION Debug )
    endif()
    
    try_compile( _result ${PROJECT_BINARY_DIR}
        ${PROJECT_SOURCE_DIR}/CMakeModules/bulletDoublePrecisionTest.cpp
        CMAKE_FLAGS
            "-DINCLUDE_DIRECTORIES:string=${BULLET_INCLUDE_DIRS}"
            "-DLINK_LIBRARIES:string=${BULLET_LIBRARIES}"
        COMPILE_DEFINITIONS
            "-DBT_USE_DOUBLE_PRECISION"
        OUTPUT_VARIABLE _buildOut
    )
    if( _result )
        message( STATUS "Bullet double precision detected. Automatically defining BT_USE_DOUBLE_PRECISION for osgBullet." )
        set( OSGBULLET_USE_DOUBLE_PRECISION ON CACHE BOOL "" FORCE )
    else()
        # Try it *without* -DBT_USE_DOUBLE_PRECISION to make sure it's single...
        set( _result )
        set( _buildOut )
        try_compile( _result ${PROJECT_BINARY_DIR}
            ${PROJECT_SOURCE_DIR}/CMakeModules/bulletDoublePrecisionTest.cpp
            CMAKE_FLAGS
                "-DINCLUDE_DIRECTORIES:string=${BULLET_INCLUDE_DIRS}"
                "-DLINK_LIBRARIES:string=${BULLET_LIBRARIES}"
            OUTPUT_VARIABLE _buildOut
        )
        if( _result )
            message( STATUS "Bullet single precision detected. Not defining BT_USE_DOUBLE_PRECISION for osgBullet." )
            set( OSGBULLET_USE_DOUBLE_PRECISION OFF CACHE BOOL "" FORCE )
        else()
            message( WARNING "Unable to determine single or double precision. Contact development staff." )
            message( "Build output follows:" )
            message( "${_buildOut}" )
        endif()
    endif()
endif()
