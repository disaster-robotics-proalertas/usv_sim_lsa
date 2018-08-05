set( _exeName testuniqifier )
MAKE_EXECUTABLE( ${_exeName}
    testuniqifier.cpp
)

add_test(
    NAME ${_exeName}
    COMMAND ${_exeName}
)
