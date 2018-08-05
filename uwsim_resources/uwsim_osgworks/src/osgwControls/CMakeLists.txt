set( LIB_NAME osgwControls )

if( BUILD_SHARED_LIBS )
    add_definitions( -DOSGWCONTROLS_LIBRARY )
endif()


set( HEADER_PATH ${PROJECT_SOURCE_DIR}/include/${LIB_NAME} )
set( LIB_PUBLIC_HEADERS
    ${HEADER_PATH}/SliderControl.h
    ${HEADER_PATH}/Export.h
)

set( _osgwControlsSourceFiles
    SliderControl.cpp
)

add_library( ${LIB_NAME}
    ${LIB_PUBLIC_HEADERS}
    ${_osgwControlsSourceFiles}
)
if( BUILD_SHARED_LIBS )
    target_link_libraries( ${LIB_NAME}
        ${OSG_LIBRARIES}
    )
endif()

set_target_properties( ${LIB_NAME} PROPERTIES VERSION ${OSGWORKS_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES SOVERSION ${OSGWORKS_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES PROJECT_LABEL "Lib ${LIB_NAME}" )

include( ModuleInstall REQUIRED )
