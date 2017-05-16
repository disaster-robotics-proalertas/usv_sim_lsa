SET( LIB_NAME osgdb_osgwTools )

ADD_OSGPLUGIN( ${LIB_NAME}
    dotosgAbsoluteModelTransform.cpp
    dotosgCameraConfigObject.cpp
    dotosgFunctionalMap.cpp
    dotosgRefID.cpp
)

INCLUDE(ModuleInstall OPTIONAL)
