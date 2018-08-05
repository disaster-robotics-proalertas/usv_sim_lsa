FIND_PATH(
    FFTSS_INCLUDE_DIR
    NAMES fftw3compat.h
    HINTS $ENV{FFTSS_DIR}/include
    PATHS /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    FFTSS_LIBRARY
    NAMES fftss libfftss
    HINTS $ENV{FFTSS_DIR}/lib
    PATHS /usr/local/lib
          /usr/lib
)

SET(FFTSS_FOUND "NO")

IF( FFTSS_INCLUDE_DIR AND FFTSS_LIBRARY )
    SET(FFTSS_FOUND "YES")
ENDIF()
