set( _exeName testypr )
MAKE_EXECUTABLE( ${_exeName}
    testypr.cpp
)

add_test(
    NAME ${_exeName}
    COMMAND ${_exeName}
)
