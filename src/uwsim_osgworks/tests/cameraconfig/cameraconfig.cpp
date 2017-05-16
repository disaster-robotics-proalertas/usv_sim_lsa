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

//
// This test verifies functionality of osgWorks' Viewer configuration utility.
// On a dual head system, run (for example):
//   cameraconfig -c viewerconfig-2x1.osg cessnafire.osg
// The viewerconfig-2x1.osg file configures the slave cameras to point slightly up
// and to the right of the center of the view.
//
// If you don't specify the -c <configfile> option, osgWorks looks for the env var
// OSGW_VIEWER_CONFIG and attempts to load the file it specifies.
//

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/ArgumentParser>
#include <osgViewer/Viewer>

#include <osgwTools/CameraConfigObject.h>


int
main( int argc, char ** argv )
{
    osg::ArgumentParser arguments( &argc, argv );

    arguments.getApplicationUsage()->setApplicationName( arguments.getApplicationName() );
    arguments.getApplicationUsage()->setDescription( arguments.getApplicationName() + " Example of savine and restoring Viewer slave camera config info." );
    arguments.getApplicationUsage()->setCommandLineUsage( arguments.getApplicationName() + " [-c <configfile>] filename ..." );

    arguments.getApplicationUsage()->addCommandLineOption( "-c <configfile>", "Specify the viewer config file to load. If not specified, looks for OSGW_VIEWER_CONFIG in the environment." );

    std::string configFile;
    while( arguments.read( "-c", configFile ) )
    {
    }

    osgViewer::Viewer viewer;
    viewer.setSceneData( osgDB::readNodeFiles( arguments ) );
    if( viewer.getSceneData() == NULL )
        viewer.setSceneData( osgDB::readNodeFile( "cow.osg" ) );

    osgwTools::configureViewer( viewer, configFile );

    return( viewer.run() );
}

