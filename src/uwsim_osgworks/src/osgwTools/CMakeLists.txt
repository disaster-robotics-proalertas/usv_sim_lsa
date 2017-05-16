set( LIB_NAME osgwTools )

if( BUILD_SHARED_LIBS )
    add_definitions( -DOSGWTOOLS_LIBRARY )
endif()


set( HEADER_PATH ${PROJECT_SOURCE_DIR}/include/${LIB_NAME} )

configure_file( "${HEADER_PATH}/Version.h.in" "${HEADER_PATH}/Version.h" @ONLY)

set( LIB_PUBLIC_HEADERS
    ${HEADER_PATH}/Export.h
    ${HEADER_PATH}/AbsoluteModelTransform.h
    ${HEADER_PATH}/CallbackSupport.h
    ${HEADER_PATH}/CameraConfigObject.h
    ${HEADER_PATH}/Capabilities.h
    ${HEADER_PATH}/CollapseLOD.h
    ${HEADER_PATH}/CountsVisitor.h
    ${HEADER_PATH}/DataLoader.h
    ${HEADER_PATH}/DecimationTestModel.h
    ${HEADER_PATH}/FBOUtils.h
    ${HEADER_PATH}/FindNamedNode.h
    ${HEADER_PATH}/ForceFlattenTransforms.h
    ${HEADER_PATH}/GeometryModifier.h
    ${HEADER_PATH}/GeometryOperation.h
    ${HEADER_PATH}/InsertRemove.h
    ${HEADER_PATH}/LODCreationNodeVisitor.h
    ${HEADER_PATH}/MultiCameraProjectionMatrix.h
    ${HEADER_PATH}/NodePathUtils.h
    ${HEADER_PATH}/NodeUtils.h
    ${HEADER_PATH}/Orientation.h
    ${HEADER_PATH}/ParallelVisitor.h
    ${HEADER_PATH}/PrimitiveSetConversion.h
    ${HEADER_PATH}/Quat.h
    ${HEADER_PATH}/ReadFile.h
    ${HEADER_PATH}/ReducerOp.h
    ${HEADER_PATH}/RefID.h
    ${HEADER_PATH}/RemoveData.h
    ${HEADER_PATH}/RemoveLOD.h
    ${HEADER_PATH}/RemoveProgram.h
    ${HEADER_PATH}/ScreenCapture.h
    ${HEADER_PATH}/SerializerSupport.h
    ${HEADER_PATH}/Shapes.h
    ${HEADER_PATH}/SimplifierOp.h
    ${HEADER_PATH}/StateSetUtils.h
    ${HEADER_PATH}/StateTrackingNodeVisitor.h
    ${HEADER_PATH}/Transform.h
    ${HEADER_PATH}/TransformUtils.h
    ${HEADER_PATH}/TransparencyUtils.h
    ${HEADER_PATH}/Trianglizer.h
    ${HEADER_PATH}/Uniqifier.h
    ${HEADER_PATH}/Version.h
)
set( _osgwToolsSourceFiles
    AbsoluteModelTransform.cpp
    CallbackSupport.cpp
    CameraConfigObject.cpp
    Capabilities.cpp
    CollapseLOD.cpp
    CountsVisitor.cpp
    FBOUtils.cpp
    FindNamedNode.cpp
    ForceFlattenTransforms.cpp
    GeometryModifier.cpp
    GeometryOperation.cpp
    InsertRemove.cpp
    LODCreationNodeVisitor.cpp
    MultiCameraProjectionMatrix.cpp
    NodePathUtils.cpp
    NodeUtils.cpp
    Orientation.cpp
    ParallelVisitor.cpp
    PluginLoader.cpp
    PrimitiveSetConversion.cpp
    Quat.cpp
    ReadFile.cpp
    ReducerOp.cpp
    RefID.cpp
    RemoveData.cpp
    RemoveLOD.cpp
    RemoveProgram.cpp
    ScreenCapture.cpp
    Shapes.cpp
    SimplifierOp.cpp
    StateSetUtils.cpp
    StateTrackingNodeVisitor.cpp
    Transform.cpp
    TransformUtils.cpp
    TransparencyUtils.cpp
    Trianglizer.cpp
    Uniqifier.cpp
    Version.cpp
)

unset( _osgwToolsHeaderFilesPre319 )
unset( _osgwToolsSourceFilesPre319 )
if( ${OPENSCENEGRAPH_VERSION} VERSION_LESS "3.1.9" )
    set( _osgwToolsHeaderFilesPre319
        ${HEADER_PATH}/DecimatorOp.h
        ${HEADER_PATH}/MeshOptimizers.h
        ${HEADER_PATH}/ShortEdgeOp.h
        ${HEADER_PATH}/TangentSpaceGeneratorDouble.h
        ${HEADER_PATH}/TangentSpaceOp.h
    )
    set( _osgwToolsSourceFilesPre319
        DecimatorOp.cpp
        MeshOptimizers.cpp
        ShortEdgeOp.cpp
        TangentSpaceGeneratorDouble.cpp
        TangentSpaceOp.cpp
    )
endif()

add_library( ${LIB_NAME}
    ${LIB_PUBLIC_HEADERS}
    ${_osgwToolsHeaderFilesPre319}
    ${_osgwToolsSourceFiles}
    ${_osgwToolsSourceFilesPre319}
)
if( BUILD_SHARED_LIBS )
    target_link_libraries( ${LIB_NAME}
        ${OSG_LIBRARIES}
        ${OPENGL_gl_LIBRARY}
    )
endif()

set_target_properties( ${LIB_NAME} PROPERTIES VERSION ${OSGWORKS_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES SOVERSION ${OSGWORKS_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES PROJECT_LABEL "Lib ${LIB_NAME}" )

include( ModuleInstall REQUIRED )
