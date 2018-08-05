set( _exeName testqueryapi )
MAKE_EXECUTABLE( ${_exeName}
    testqueryapi.cpp
)

add_test(
    NAME ${_exeName}
    COMMAND ${_exeName}
)
