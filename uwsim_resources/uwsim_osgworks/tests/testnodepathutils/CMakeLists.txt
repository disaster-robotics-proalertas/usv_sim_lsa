set( _exeName testnodepathutils )
MAKE_EXECUTABLE( ${_exeName}
    testnodepathutils.cpp
)

add_test(
    NAME ${_exeName}
    COMMAND ${_exeName}
)
