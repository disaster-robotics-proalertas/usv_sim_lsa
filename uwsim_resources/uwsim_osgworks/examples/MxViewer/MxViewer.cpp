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

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgwMx/MxEventHandler.h>


int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );

    osg::ref_ptr< osg::Node > root = osgDB::readNodeFiles( arguments );
    if( !( root.valid() ) )
    {
        root = osgDB::readNodeFile( "dumptruck.osg" );
        if( !( root.valid() ) )
        {
            osg::notify( osg::FATAL ) << "Can't load input file." << std::endl;
            return( 1 );
        }
    }

    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 30, 30, 800, 450 );
    viewer.setSceneData( root.get() );
    viewer.addEventHandler( new osgViewer::StatsHandler );


    /* Set up the MxEventHandler. This is not a typical OSG CameraManipulator,
    which can change only the view matrix. The MxEventHandler seperates the
    act of receiving events from the act of setting the Camera view and projection
    matrices. */
    osg::ref_ptr< osgwMx::MxEventHandler > viewingHandler = new osgwMx::MxEventHandler;

    // Must specify the scene graph being viewed; used internally for zillions of things.
    viewingHandler->setSceneData( root.get() );
    // Add the MxEventHandler as a standard viewer EventHandler.
    viewer.addEventHandler( viewingHandler.get() );
    // Get the update callback from the MxEventHandler and attach it as an update
    // callback to the viewer's internal Camera node. This will update the view and
    // projection matrices each frame.
    viewer.getCamera()->setUpdateCallback( viewingHandler->getMatrixCallback() );
    // We'll modify the Camera node, so mark it as DYNAMIC for thread safety.
    viewer.getCamera()->setDataVariance( osg::Object::DYNAMIC );

    // Do not use viewer.run(), which automatically adds a camera manipulator
    // if one doesn't already exist.
    while( !viewer.done() )
        viewer.frame();
}


/** \page mxviewer The MxViewer Example
MxViewer demonstrates use of the osgwMx library for view control.

For keyboard and mouse controls, see the osgwMx::MxEventHandler documentation.
*/
