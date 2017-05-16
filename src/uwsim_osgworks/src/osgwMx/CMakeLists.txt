set( LIB_NAME osgwMx )

if( BUILD_SHARED_LIBS )
    add_definitions( -DOSGWMX_LIBRARY )
endif()


set( HEADER_PATH ${PROJECT_SOURCE_DIR}/include/${LIB_NAME} )

set( DX_DEPENDENT_HEADERS )
set( DX_DEPENDENT_SRC )
if( DIRECTINPUT_FOUND )
	add_definitions(-DDIRECTINPUT_ENABLED)
    include_directories( ${DIRECTINPUT_INCLUDE_DIR} )
    set( DX_DEPENDENT_HEADERS
        ${HEADER_PATH}/MxGamePadDX.h
    )
    set( DX_DEPENDENT_SRC
        MxGamePadDX.cpp
    )
endif()

set( VRPN_DEPENDENT_HEADERS )
set( VRPN_DEPENDENT_SRC )
if( VRPN_FOUND )
	add_definitions(-DVRPN_ENABLED)
    include_directories( ${VRPN_INCLUDES} )
    set( VRPN_DEPENDENT_HEADERS
        ${HEADER_PATH}/MxSpaceBallVRPN.h
    )
    set( VRPN_DEPENDENT_SRC
        MxSpaceBallVRPN.cpp
    )
endif() #VRPN_FOUND


set( LIB_PUBLIC_HEADERS
    ${DX_DEPENDENT_HEADERS}
    ${VRPN_DEPENDENT_HEADERS}
    ${HEADER_PATH}/Export.h
    ${HEADER_PATH}/CameraUpdateCallback.h
    ${HEADER_PATH}/MxCore.h
    ${HEADER_PATH}/MxEventHandler.h
    ${HEADER_PATH}/MxGamePad.h
    ${HEADER_PATH}/MxSpaceBall.h
    ${HEADER_PATH}/MxUtils.h
)

set( _osgwMxSourceFiles
    ${DX_DEPENDENT_SRC}
    ${VRPN_DEPENDENT_SRC}
    CameraUpdateCallback.cpp
    MxCore.cpp
    MxEventHandler.cpp
    MxGamePad.cpp
    MxSpaceBall.cpp
    MxUtils.cpp
)


add_library( ${LIB_NAME}
    ${LIB_PUBLIC_HEADERS}
    ${_osgwMxSourceFiles}
)

find_package( OpenGL REQUIRED )
include_directories(
    ${OPENGL_INCLUDE_DIR}
)

if( BUILD_SHARED_LIBS )
    # Shared libs must explicitly resolve all symbols.
    # Link against our dependency: OSG.
    target_link_libraries( ${LIB_NAME}
        ${OSG_LIBRARIES}
        ${OPENGL_LIBRARIES}
    )

    # osgwMx depends on osgwTools
    target_link_libraries( ${LIB_NAME}
        optimized "osgwTools"
        debug "osgwTools"
    )

    if( DIRECTINPUT_FOUND )
       target_link_libraries( ${LIB_NAME} ${DIRECTINPUT_LIBRARIES} )
    endif()
    
    if( VRPN_FOUND )
       target_link_libraries( ${LIB_NAME} ${VRPN_LIBRARY} )
    endif()
    

endif()

set_target_properties( ${LIB_NAME} PROPERTIES VERSION ${OSGWORKS_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES SOVERSION ${OSGWORKS_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES PROJECT_LABEL "Lib ${LIB_NAME}" )

include( ModuleInstall REQUIRED )
