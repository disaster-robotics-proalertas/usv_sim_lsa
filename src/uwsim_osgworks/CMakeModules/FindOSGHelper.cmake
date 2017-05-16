
set( _osgComponents
    osgGA osgText osgViewer osgSim osgDB osgUtil osg OpenThreads
)

# Macro to force the stock FindOpenSceneGraph.cmake script to
# search again for OSG.
macro( unFindOSG )
#    message( STATUS "In unFindOSG" )

    foreach( currentLibIn ${_osgComponents} )
        string( TOUPPER ${currentLibIn} currentLib )
        unset( "${currentLib}_INCLUDE_DIR" CACHE )
        unset( "${currentLib}_LIBRARIES" CACHE )
        unset( "${currentLib}_LIBRARY" CACHE )
        unset( "${currentLib}_LIBRARY_DEBUG" CACHE )
    endforeach() 
endmacro( unFindOSG )


# What type of OSG should we look for? This is a combo box
# supporting three types of OSG installations:
#  * Default installation (in which the stock FindOpenSceneGraph.cmake script does all the work)
#  * Alternate Install Location (user must set the OSGInstallLocation variable)
#  * Source And Build Tree (user must supply both the OSGSourceRoot and OSGBuildRoot variables)
set( OSGInstallType "Default Installation" CACHE STRING "Type of OSG install: 'Default Installation', 'Alternate Install Location', or 'Source And Build Tree'." )
set_property( CACHE OSGInstallType PROPERTY STRINGS "Default Installation" "Alternate Install Location" "Source And Build Tree" )

# We need to detect when the user changes the OSG install type
# or any of the related directory variables, so that we'll know
# to call unFindOSG() and force the stock OSG script to search
# again. To do this, we save the last set value of these variables
# in the CMake cache as internal (hidden) variables.
if( NOT DEFINED _lastOSGInstallType )
    set( _lastOSGInstallType "empty" CACHE INTERNAL "" )
endif()
if( NOT DEFINED _lastOSGInstallLocation )
    set( _lastOSGInstallLocation "empty" CACHE INTERNAL "" )
endif()
if( NOT DEFINED _lastOSGSourceRoot )
    set( _lastOSGSourceRoot "empty" CACHE INTERNAL "" )
endif()
if( NOT DEFINED _lastOSGBuildRoot )
    set( _lastOSGBuildRoot "empty" CACHE INTERNAL "" )
endif()

if( NOT DEFINED OSGInstallLocation )
    set( OSGInstallLocation "Please specify" )
endif()
if( NOT DEFINED OSGSourceRoot )
    set( OSGSourceRoot "Please specify" )
endif()
if( NOT DEFINED OSGBuildRoot )
    set( OSGBuildRoot "Please specify" )
endif()


# If the user has changed the OSG install type combo box
# (or it's a clean cache), then set or unset the our related
# OSG directory search variables.
if( NOT ( ${OSGInstallType} STREQUAL ${_lastOSGInstallType} ) )
#    message( STATUS "NOT ( ${OSGInstallType} STREQUAL ${_lastOSGInstallType} )" )

    if( OSGInstallType STREQUAL "Default Installation" )
        # Remove our helper variables and tell the stock script to search again.
        unset( OSGInstallLocation CACHE )
        unset( OSGSourceRoot CACHE )
        unset( OSGBuildRoot CACHE )
    elseif( OSGInstallType STREQUAL "Alternate Install Location" )
        # Enable just the OSGInstallLocation helper variable.
        set( OSGInstallLocation "Please specify" CACHE PATH "Root directory where OSG is installed" )
        unset( OSGSourceRoot CACHE )
        unset( OSGBuildRoot CACHE )
    elseif( OSGInstallType STREQUAL "Source And Build Tree" )
        # Enable the OSGSourceRoot and OSGBuildRoot helper variables.
        unset( OSGInstallLocation CACHE )
        set( OSGSourceRoot "Please specify" CACHE PATH "Root directory of OSG source tree" )
        set( OSGBuildRoot "Please specify" CACHE PATH "Root directory of OSG build tree" )
    endif()
endif()

# Look for conditions that require us to find OSG again.
set( _needToFindOSG FALSE )
if( OSGInstallType STREQUAL "Default Installation" )
    if( NOT ( ${OSGInstallType} STREQUAL ${_lastOSGInstallType} ) )
#        message( STATUS "Need to find: case A" )
        set( _needToFindOSG TRUE )
    endif()
elseif( OSGInstallType STREQUAL "Alternate Install Location" )
    if( NOT ( "${OSGInstallLocation}" STREQUAL "${_lastOSGInstallLocation}" ) )
#        message( STATUS "Need to find: case B" )
        set( _needToFindOSG TRUE )
    endif()
elseif( OSGInstallType STREQUAL "Source And Build Tree" )
    if( ( NOT ( "${OSGSourceRoot}" STREQUAL "${_lastOSGSourceRoot}" ) ) OR
        ( NOT ( "${OSGBuildRoot}" STREQUAL "${_lastOSGBuildRoot}" ) ) )
#        message( STATUS "Need to find: cade C" )
        set( _needToFindOSG TRUE )
    endif()
endif()
if( _needToFindOSG )
    unFindOSG()
    set( _lastOSGInstallType ${OSGInstallType} CACHE INTERNAL "" FORCE )
    set( _lastOSGInstallLocation ${OSGInstallLocation} CACHE INTERNAL "" FORCE )
    set( _lastOSGSourceRoot ${OSGSourceRoot} CACHE INTERNAL "" FORCE )
    set( _lastOSGBuildRoot ${OSGBuildRoot} CACHE INTERNAL "" FORCE )
endif()



# Save internal variables for later restoration
set( CMAKE_PREFIX_PATH_SAVE ${CMAKE_PREFIX_PATH} )
set( CMAKE_LIBRARY_PATH_SAVE ${CMAKE_LIBRARY_PATH} )

set( CMAKE_PREFIX_PATH
    ${OSGInstallLocation}
    ${OSGSourceRoot}
    ${OSGBuildRoot} )
set( CMAKE_LIBRARY_PATH
    ${OSGInstallLocation}
    ${OSGBuildRoot} )

set( OpenSceneGraph_DEBUG 0 )
set( OpenSceneGraph_MARK_AS_ADVANCED 1 )
find_package( OpenSceneGraph 2.6.1 REQUIRED COMPONENTS ${_osgComponents} )
if( OSG_FOUND )
    foreach( currentLibIn ${_osgComponents} )
        string( TOUPPER ${currentLibIn} currentLib )
        list( APPEND _tempLibraries ${${currentLib}_LIBRARIES} )
        list( APPEND _tempIncludeDirs ${${currentLib}_INCLUDE_DIR} )
    endforeach()
    list( REMOVE_DUPLICATES _tempIncludeDirs )
    set( OSG_LIBRARIES ${_tempLibraries} )
    set( OSG_INCLUDE_DIRS ${_tempIncludeDirs} )

    if( OSGInstallType STREQUAL "Source And Build Tree" )
        # Hm, the OSGBuildRoot seems to be left out of the include path,
        # even though that's where the Config headers are located. Add it:
        set( OSG_INCLUDE_DIRS ${OSG_INCLUDE_DIRS} "${OSGBuildRoot}/include" )
    endif()
endif()


# Restore internal variables
set( CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH_SAVE} )
set( CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH_SAVE} )
