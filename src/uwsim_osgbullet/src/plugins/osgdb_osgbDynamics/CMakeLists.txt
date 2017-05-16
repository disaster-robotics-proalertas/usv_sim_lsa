set( LIB_NAME osgdb_osgbdynamics )

_osgBulletPlugin( ${LIB_NAME}
    dotosgArticulationRecord.cpp
    dotosgConstraints.cpp
    dotosgCreationRecord.cpp
    dotosgMatrixIO.cpp
    dotosgMatrixIO.h
    dotosgPhysicsData.cpp
)

INCLUDE( ModuleInstall REQUIRED )
