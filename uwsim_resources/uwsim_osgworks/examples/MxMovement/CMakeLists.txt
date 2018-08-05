if( DIRECTINPUT_FOUND)
    add_definitions(-DDIRECTINPUT_ENABLED)
endif()

if( VRPN_FOUND)
    add_definitions(-DVRPN_ENABLED)
    include_directories( ${VRPN_INCLUDES} )
endif()

if( DIRECTINPUT_FOUND OR VRPN_FOUND)
    MAKE_EXECUTABLE( MxMovement
        MxMovementExample.cpp
    )
endif()
