set( LIB_NAME osgbInteraction )
set( HEADER_PATH ${PROJECT_SOURCE_DIR}/include/${LIB_NAME} )
add_definitions( -DOSGBINTERACTION_LIBRARY )

if( OSGBULLET_P5_SUPPORT )
    set( P5_HEADER ${HEADER_PATH}/p5support.h )
    set( P5_SOURCE p5support.cpp )
else()
    unset( P5_HEADER )
    unset( P5_SOURCE )
endif()


set( LIB_PUBLIC_HEADERS
    ${P5_HEADER}
    ${HEADER_PATH}/Export.h
    ${HEADER_PATH}/ArticulationRecord.h
    ${HEADER_PATH}/DragHandler.h
    ${HEADER_PATH}/LaunchHandler.h
    ${HEADER_PATH}/GestureHandler.h
    ${HEADER_PATH}/HandNode.h
    ${HEADER_PATH}/HandTestEventHandler.h
    ${HEADER_PATH}/SaveRestoreHandler.h
)

set( _osgBulletSourceFiles
    ${P5_SOURCE}
    ArticulationRecord.cpp
    DragHandler.cpp
    LaunchHandler.cpp
    GestureHandler.cpp
    HandNode.cpp
    HandTestEventHandler.cpp
    SaveRestoreHandler.cpp
)

add_library( ${LIB_NAME}
    ${LIB_PUBLIC_HEADERS}
    ${_osgBulletSourceFiles}
)
if( BUILD_SHARED_LIBS )
    target_link_libraries( ${LIB_NAME}
        ${OSGWORKS_LIBRARIES}
        ${OSG_LIBRARIES}
        ${BULLET_LIBRARIES}
        ${OSGBULLET_P5_LIBRARY}
    )

    # osgbDynamics depends on osgbCollision
    target_link_libraries( ${LIB_NAME}
        optimized "osgbDynamics"
        debug "osgbDynamics"
        optimized "osgbCollision"
        debug "osgbCollision"
        ${OSGBULLET_P5_LIBRARY}
    )
endif()

set_target_properties( ${LIB_NAME} PROPERTIES VERSION ${OSGBULLET_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES SOVERSION ${OSGBULLET_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES PROJECT_LABEL "Lib ${LIB_NAME}" )

include( ModuleInstall REQUIRED )
