set( LIB_NAME osgdb_osgobjects )

ADD_OSGPLUGIN( ${LIB_NAME}
    ReaderWriterOSGObjects.cpp
    ReaderWriterOSGObjects.h
)

include( ModuleInstall OPTIONAL )
