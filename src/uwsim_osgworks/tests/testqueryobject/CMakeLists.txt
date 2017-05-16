set( _exeName testqueryobject )
MAKE_EXECUTABLE( ${_exeName}
    testqueryobject.cpp
)

add_test(
    NAME ${_exeName}
    COMMAND ${_exeName}
)
