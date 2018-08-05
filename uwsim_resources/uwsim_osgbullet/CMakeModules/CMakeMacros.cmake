if( WIN32 )
    set( CMAKE_DEBUG_POSTFIX d )
endif()

macro( link_internal TRGTNAME )
    foreach( LINKLIB ${ARGN} )
        target_link_libraries( ${TRGTNAME} optimized "${LINKLIB}" debug "${LINKLIB}" )
    endforeach()
endmacro()


macro( _osgBulletPlugin TRGTNAME )
    if( BUILD_SHARED_LIBS )
        add_library( ${TRGTNAME} MODULE ${ARGN} )

        # TBD. This is a shortcut. It means any app that wants to be
        # collision-only, for example, and also use the dot OSG support,
        # must link with all three libs. FIXME.
        link_internal( ${TRGTNAME}
            osgbInteraction
            osgbDynamics
            osgbCollision
        )
        target_link_libraries( ${TRGTNAME}
            ${OSGWORKS_LIBRARIES}
            ${OSG_LIBRARIES}
        )
    else()
        add_library( ${TRGTNAME} STATIC ${ARGN} )
    endif()

    if( WIN32 )
        set_target_properties( ${TRGTNAME} PROPERTIES DEBUG_POSTFIX d )
    endif()
    set_target_properties( ${TRGTNAME} PROPERTIES PREFIX "" )
    set_target_properties( ${TRGTNAME} PROPERTIES PROJECT_LABEL "Plugin ${TRGTNAME}" )
endmacro()


macro( _osgBulletMakeInteractionExe _exeName )
    set( _osgBulletLibs
        "osgbInteraction;osgbDynamics;osgbCollision"
    )
    set( _bulletLibs
        "${BULLET_LIBRARIES}"
        "${OSGBULLET_P5_LIBRARY}"
    )
    _osgBulletMakeExeInternal( ${_exeName} "${_osgBulletLibs}" "${_bulletLibs}" ${ARGN} )
endmacro()

macro( _osgBulletMakeDynamicsExe _exeName )
    set( _osgBulletLibs
        "osgbDynamics;osgbCollision"
    )
    set( _bulletLibs
        "${BULLET_LIBRARIES}"
    )
    _osgBulletMakeExeInternal( ${_exeName} "${_osgBulletLibs}" "${_bulletLibs}" ${ARGN} )
endmacro()

macro( _osgBulletMakeCollisionExe _exeName )
    set( _osgBulletLibs
        "osgbCollision"
    )
    if( BULLET_COLLISION_LIBRARY_DEBUG AND BULLET_MATH_LIBRARY_DEBUG )
        set( _bulletLibs
            optimized ${BULLET_COLLISION_LIBRARY} debug ${BULLET_COLLISION_LIBRARY_DEBUG}
            optimized ${BULLET_MATH_LIBRARY} debug ${BULLET_MATH_LIBRARY_DEBUG}
        )
    else()
        set( _bulletLibs
            ${BULLET_COLLISION_LIBRARY}
            ${BULLET_MATH_LIBRARY}
        )
    endif()
    _osgBulletMakeExeInternal( ${_exeName} "${_osgBulletLibs}" "${_bulletLibs}" ${ARGN} )
endmacro()

macro( _osgBulletMakeExeInternal _exeName _osgBulletLibs _bulletLibs )
    add_executable( ${_exeName} ${ARGN} )
    if( WIN32 )
        set_target_properties( ${_exeName} PROPERTIES DEBUG_POSTFIX d )
    endif()

    link_internal( ${_exeName}
        ${_osgBulletLibs}
    )
    target_link_libraries( ${_exeName}
        ${OSGWORKS_LIBRARIES}
        ${OSG_LIBRARIES}
        ${_bulletLibs}
    )
    if( ${CATEGORY} STREQUAL "App" )
        install(
            TARGETS ${_exeName}
            RUNTIME DESTINATION bin COMPONENT osgbullet
        )
    else()
        install(
            TARGETS ${_exeName}
            RUNTIME DESTINATION share/${CMAKE_PROJECT_NAME}/bin COMPONENT osgbullet
        )
    endif()
    set_target_properties( ${_exeName} PROPERTIES PROJECT_LABEL "${CATEGORY} ${_exeName}" )
endmacro()
