set( _imageFileList
    geom-edgecollapse.jpg
    geom-halfedgecollapse.jpg
    geom-reducergroup.jpg
    geom-reducerremove.jpg
    osgwbvv00.jpg
)

foreach( curImage ${_imageFileList} )
    configure_file( ${curImage}
        ${PROJECT_BINARY_DIR}/doc/html/${curImage} COPYONLY )
endforeach()
