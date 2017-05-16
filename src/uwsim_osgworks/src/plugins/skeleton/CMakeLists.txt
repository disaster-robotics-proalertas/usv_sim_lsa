SET( LIB_NAME osgdb_skeleton )

ADD_OSGPLUGIN( ${LIB_NAME}
    ReaderWriterSkeleton.cpp
    ReaderWriterSkeleton.h
)

INCLUDE(ModuleInstall OPTIONAL)
