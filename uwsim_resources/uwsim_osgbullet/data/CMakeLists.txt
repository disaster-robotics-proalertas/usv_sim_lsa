# Inputs. If we rewrite this as a function or macro later,
# we need these as parameters:
#   _srcDir - Directory of files to install.
#   _destDir - Relative destincation install directory.
#   -exclude - List of files that should not be installed.
set( _srcDir ${PROJECT_SOURCE_DIR}/data )
set( _destDir share/${CMAKE_PROJECT_NAME}/data )
set( _exclude "CMakeLists.txt" )

# Get the raw list of files in _srcDir.
file( GLOB fileList RELATIVE ${_srcDir} ${_srcDir}/* )

foreach( trgtFile ${fileList} )
    # If a file is either a directory or on the _exclude list,
    # don't install it. (Remove it from fileList.)
    list( FIND _exclude ${trgtFile} excludeFile )
    if(    NOT ( ${excludeFile} EQUAL -1 ) OR
           IS_DIRECTORY ${_srcDir}/${trgtFile}   )
        list( REMOVE_ITEM fileList ${trgtFile} )
    endif()
endforeach()

# Debug: display the final list of files to install.
#message( STATUS "--- ${fileList}" )

install( FILES ${fileList}
    DESTINATION ${_destDir}
    COMPONENT libosgworks
)
