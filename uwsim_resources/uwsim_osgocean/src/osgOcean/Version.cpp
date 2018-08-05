#include <osgOcean/Version>
#include <string>
#include <stdio.h>

const char* osgOceanGetVersion()
{
    static char ocean_version[256];
    static int ocean_version_init = 1;
    
    if( ocean_version_init )
    {
        if( OSGOCEAN_VERSION_REVISION == 0 )
            sprintf( ocean_version, "%d.%d.%d", OSGOCEAN_VERSION_MAJOR, OSGOCEAN_VERSION_MINOR, OSGOCEAN_VERSION_RELEASE );
        else
            sprintf( ocean_version, "%d.%d.%d-%d", OSGOCEAN_VERSION_MAJOR, OSGOCEAN_VERSION_MINOR, OSGOCEAN_VERSION_RELEASE, OSGOCEAN_VERSION_REVISION );

        ocean_version_init = 0;
    }
    
    return ocean_version;
}

const char* osgOceanGetLibraryName()
{
    return "osgOcean Library";
}