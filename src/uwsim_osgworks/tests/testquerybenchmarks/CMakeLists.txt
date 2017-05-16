set( _exeName testquerybenchmarks )
MAKE_EXECUTABLE( ${_exeName}
    testquerybenchmarks.cpp
)

add_test(
    NAME ${_exeName}
    COMMAND ${_exeName}
)
