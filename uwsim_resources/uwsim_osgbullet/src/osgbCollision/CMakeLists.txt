set( LIB_NAME osgbCollision )
set( HEADER_PATH ${PROJECT_SOURCE_DIR}/include/${LIB_NAME} )
add_definitions( -DOSGBCOLLISION_LIBRARY )

configure_file( "${HEADER_PATH}/Version.h.in" "${HEADER_PATH}/Version.h" @ONLY)

set( LIB_PUBLIC_HEADERS
    ${HEADER_PATH}/BoundingCone.h
    ${HEADER_PATH}/BoundingCylinder.h
    ${HEADER_PATH}/Chart.h
    ${HEADER_PATH}/CollectVerticesVisitor.h
    ${HEADER_PATH}/CollisionShapes.h
    ${HEADER_PATH}/ComputeCylinderVisitor.h
    ${HEADER_PATH}/ComputeShapeVisitor.h
    ${HEADER_PATH}/ComputeTriMeshVisitor.h
    ${HEADER_PATH}/GLDebugDrawer.h
    ${HEADER_PATH}/RefBulletObject.h
    ${HEADER_PATH}/Utils.h
    ${HEADER_PATH}/Version.h
    ${HEADER_PATH}/VertexAggOp.h
    ${HEADER_PATH}/Export.h
)

set( _osgBulletSourceFiles
    BoundingCone.cpp
    BoundingCylinder.cpp
    Chart.cpp
    CollectVerticesVisitor.cpp
    CollisionShapes.cpp
    ComputeCylinderVisitor.cpp
    ComputeShapeVisitor.cpp
    ComputeTriMeshVisitor.cpp
    GLDebugDrawer.cpp
    Utils.cpp
    VertexAggOp.cpp
    Version.cpp
)

add_library( ${LIB_NAME}
    ${LIB_PUBLIC_HEADERS}
    ${_osgBulletSourceFiles}
)
if( BUILD_SHARED_LIBS )
    if( BULLET_COLLISION_LIBRARY_DEBUG AND BULLET_MATH_LIBRARY_DEBUG )
        set( _localBulletLibs
            optimized ${BULLET_COLLISION_LIBRARY} debug ${BULLET_COLLISION_LIBRARY_DEBUG}
            optimized ${BULLET_MATH_LIBRARY} debug ${BULLET_MATH_LIBRARY_DEBUG}
        )
    else()
        set( _localBulletLibs
            ${BULLET_COLLISION_LIBRARY}
            ${BULLET_MATH_LIBRARY}
        )
    endif()

    target_link_libraries( ${LIB_NAME}
        ${OSGWORKS_LIBRARIES}
        ${OSG_LIBRARIES}
        ${_localBulletLibs}
    )
endif()

set_target_properties( ${LIB_NAME} PROPERTIES VERSION ${OSGBULLET_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES SOVERSION ${OSGBULLET_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES PROJECT_LABEL "Lib ${LIB_NAME}" )

include( ModuleInstall REQUIRED )
