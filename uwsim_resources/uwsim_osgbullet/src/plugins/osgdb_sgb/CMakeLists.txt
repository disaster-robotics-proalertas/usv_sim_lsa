set( LIB_NAME osgdb_sgb )

_osgBulletPlugin( ${LIB_NAME}
    ReaderWriterSGB.cpp
    ReaderWriterSGB.h
)

include( ModuleInstall REQUIRED )
