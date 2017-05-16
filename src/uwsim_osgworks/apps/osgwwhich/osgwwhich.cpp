/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgWorks is (C) Copyright 2009-2012 by Kenneth Mark Bryden
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License version 2.1 as published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *************** <auto-copyright.pl END do not edit this line> ***************/

#include "osgwTools/Version.h"
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osg/Version>
#include <osg/ArgumentParser>

#include <osg/io_utils>
#include <string>



std::string
createLibraryName( std::string simpleName )
{
#if defined( WIN32 ) || defined( _WIN32 )
    return( std::string( "osg" ) +
        std::string( osgGetSOVersion() ) +
        std::string( "-" ) +
        simpleName +
        std::string( ".dll" ) );
#else
    return( std::string( "lib" ) +
        simpleName +
        std::string( "." ) +
        std::string( osgGetVersion() ) +
        std::string( ".dylib" ) );
#endif
}

void
locatePlugin( const std::string& target )
{
    std::string fullName = osgDB::findLibraryFile( target );
    if( fullName == std::string( "" ) )
    {
        osg::notify( osg::ALWAYS ) << "Can't find: " << target << std::endl;
        return;
    }
    osg::notify( osg::ALWAYS ) << fullName << std::endl;
}

void
locateSharedLibrary( const std::string& target )
{

    std::string fullName( osgDB::findLibraryFile( target ) );
#if 0 // future apple support? defined( __APPLE__ )
    if( fullName == std::string( "" ) )
    {
        // findLibraryFile is kind of hosed on OS X because it
        // always searches the plugin paths. To fix, we must explicitly
        // check the non-plugin paths.
        osgDB::FilePathList filePath;
        std::string path;
        if( getenv( "PATH" ) != NULL )
            path = std::string( getenv( "PATH" ) );
        if( getenv( "LD_LIBRARY_PATH" ) != NULL )
            path += std::string( getenv( "LD_LIBRARY_PATH" ) );
        osgDB::convertStringPathIntoFilePathList( path, filePath );
        fullName = osgDB::findFileInPath( target, filePath );
    }
#endif
    if( fullName == std::string( "" ) )
    {
        // We didn't find this as a library.
        // Add the plugin prefix and check in that locaion.

        // From osgDB::Registry::createLibraryNameForExtension
        const std::string prepend = std::string("osgPlugins-")+std::string(osgGetVersion())+std::string("/");

        fullName = osgDB::findLibraryFile( prepend + target );
        if( fullName == std::string( "" ) )
        {
            osg::notify( osg::ALWAYS ) << "Can't find: " << target << std::endl;
            return;
        }
    }
    osg::notify( osg::ALWAYS ) << fullName << std::endl;
}

void
locateDataFile( const std::string& target )
{
    const std::string fullName( osgDB::findDataFile( target ) );
    if( fullName == std::string( "" ) )
        osg::notify( osg::ALWAYS ) << "Can't find: " << target << std::endl;
    else
        osg::notify( osg::ALWAYS ) << fullName << std::endl;
}


int
main( int argc,
      char ** argv )
{
    osg::ArgumentParser arguments( &argc, argv );

    arguments.getApplicationUsage()->setApplicationName( arguments.getApplicationName() );
    arguments.getApplicationUsage()->setDescription( arguments.getApplicationName() + " locates data files and shared libraries using OSG's search algorithms." );
    arguments.getApplicationUsage()->setCommandLineUsage( arguments.getApplicationName() + " [options] [filename [...] ]" );

    arguments.getApplicationUsage()->addCommandLineOption( "-v/--version", "Display the osgWorks version string." );
    arguments.getApplicationUsage()->addCommandLineOption( "-p/--plugin <ext>", "Locate a plugin to support the extension <ext>, e.g., \"flt\"." );
    arguments.getApplicationUsage()->addCommandLineOption( "-l/--library <libname>", "Locate the library indicated with <libName>, e.g., \"osgUtil\"." );

    if( arguments.read( "-h" ) || arguments.read( "--help" ) )
    {
        osg::notify( osg::ALWAYS ) << arguments.getApplicationUsage()->getDescription() << std::endl;
        arguments.getApplicationUsage()->write( osg::notify( osg::ALWAYS ), osg::ApplicationUsage::COMMAND_LINE_OPTION );
        return 1;
    }

    if( arguments.read( "-v" ) || arguments.read( "--version" ) )
    {
        osg::notify( osg::ALWAYS ) << osgwTools::getVersionString() << std::endl << std::endl;
    }

    std::string param;
    if( arguments.read( "-p", param ) || arguments.read( "--plugin", param ) )
    {
        std::string plugin( osgDB::Registry::instance()->createLibraryNameForExtension( param ) );
        osg::notify( osg::ALWAYS ) << "Using plugin name: \"" << plugin << "\"." << std::endl;
        locateSharedLibrary( plugin );
    }    

    if( arguments.read( "-l", param ) || arguments.read( "--library", param ) )
    {
        std::string library( createLibraryName( param ) );
        locateSharedLibrary( library );
    }    


    int idx;
    for( idx=1; idx < arguments.argc(); idx++ )
    {
        if( !arguments.isOption( idx ) )
        {
            // Argument is not an option. Try to locate it.
            const std::string target( arguments[ idx ] );
            const std::string ext( osgDB::getLowerCaseFileExtension( target ) );
            if( ( ext == std::string( "dll" ) ) ||
                ( ext == std::string( "so" ) ) ||
                ( ext == std::string( "dylib" ) ) )
                locateSharedLibrary( target );
            else
                locateDataFile( target );
        }
    }
    
    return( 0 );
}



/** \page osgwwhich The osgwwhich Application
osgwwhich locates OSG data files and shared libraries.

osgwwhich uses the same search algorithm employed by OSG to find data files and
shared libraries, and then displays the full path of the specified file. This is
useful when you have multiple data files with the same name, as osgwwhich tells
you the location of the file that OSG would load. It's also useful when you have
multiple copies of OSG installed on your system, as osgwwhich tells you the path
to the OSG library or plugin.

For example, the following command shows where OSG will find "cow.osg" (a part of
the OpenSceneGraph?-Data sample data set):

\code
C:\Users\pmartz>osgwwhich cow.osg
C:\Projects\OSG\OSG-Data\cow.osg
\endcode

Note that this works with the OSG notify level. If you wanted to see all the
locations that OSG searches before it finds your data file, set the OSG_NOTIFY_LEVEL
accordingly.

\code
C:\Users\pmartz>set OSG_NOTIFY_LEVEL=DEBUG
C:\Users\pmartz>osgwwhich tetra.osg
itr='C:\Projects\OSG\OSG-Data' FindFileInPath() : trying C:\Projects\OSG\OSG-Data\tetra.osg ...
itr='C:\Projects\Physics\osgBullet\data' FindFileInPath() : trying C:\Projects\Physics\osgBullet\data\tetra.osg ...
FindFileInPath() : USING C:\Projects\Physics\osgBullet\data\tetra.osg
C:\Projects\Physics\osgBullet\data\tetra.osg
\endcode

The following command locates the OSG OpenFlight plugin: 

\code
C:\Users\pmartz>osgwwhich osgdb_openflight.dll
C:\Program Files\OpenSceneGraph\bin\osgPlugins-2.8.2\osgdb_openflight.dll
\endcode

The following command displays the osgWorks version number, and also
locates the osgUtil library: 

\code
C:\Users\pmartz>osgwwhich -v osg55-osgUtil.dll
osgWorks version 1.0.0 (10000).
C:\Program Files\OpenSceneGraph\bin\osg55-osgUtil.dll
\endcode

If you don't know the library or plugin name format, use the -l and -p
options with the basic library name or plugin extension. For example,
the following command finds the osgUtil library:

\code
C:\Users\pmartz>osgwwhich -l osgUtil
C:\Program Files\OpenSceneGraph\bin\osg55-osgUtil.dll
\endcode

And the following command finds the DDS plugin: 

\code
C:\Users\pmartz>osgwwhich -p dds
Using plugin name: "osgPlugins-2.8.2/osgdb_dds.dll".
C:\Program Files\OpenSceneGraph\bin\osgPlugins-2.8.2\osgdb_dds.dll
\endcode

\bug osgwwhich has known issues finding dynamic libraries on OS X.

*/

