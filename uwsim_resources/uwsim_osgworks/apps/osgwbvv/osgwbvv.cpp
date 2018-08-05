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
#include "osgwTools/Shapes.h"

#include <osgDB/ReadFile>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osg/ArgumentParser>
#include <osg/ComputeBoundsVisitor>
#include <osg/BoundingSphere>
#include <osg/MatrixTransform>
#include <osg/AutoTransform>
#include <osg/PolygonMode>
#include <osg/Point>
#include <osg/BlendFunc>

#include <osg/io_utils>



osg::Node* makeOrigin()
{
    osg::ref_ptr< osg::AutoTransform > at = new osg::AutoTransform;
    at->setAutoRotateMode( osg::AutoTransform::ROTATE_TO_SCREEN );
    at->setAutoScaleToScreen( true );

    osg::ref_ptr< osg::Geode > geode = new osg::Geode;
    at->addChild( geode.get() );

    osg::StateSet* ss = geode->getOrCreateStateSet();
    ss->setRenderBinDetails( 0x7fffffff, "RenderBin" );
    ss->setMode( GL_DEPTH_TEST, osg::StateAttribute::OFF );
    ss->setAttributeAndModes( new osg::Point( 6. ) );

    const float radius = 50.f;
    const unsigned int subdivisions = 32;
    osg::Vec3 orientation = osg::Vec3( 0., 0., 1. );
    geode->addDrawable( osgwTools::makeWireCircle( radius, subdivisions, orientation ) );

    osg::ref_ptr< osg::Geometry > geom = new osg::Geometry;
    osg::Vec3Array* v = new osg::Vec3Array;
    geom->setVertexArray( v );
    v->push_back( osg::Vec3( 0., 0., 0. ) );
    v->push_back( osg::Vec3( -radius, 0., 0. ) );
    v->push_back( osg::Vec3( radius, 0., 0. ) );
    v->push_back( osg::Vec3( 0., -radius, 0. ) );
    v->push_back( osg::Vec3( 0., radius, 0. ) );
    geom->addPrimitiveSet( new osg::DrawArrays( GL_POINTS, 0, 1 ) );
    geom->addPrimitiveSet( new osg::DrawArrays( GL_LINES, 1, 4 ) );
    geode->addDrawable( geom.get() );

    return( at.release() );
}


int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );

    arguments.getApplicationUsage()->setApplicationName( arguments.getApplicationName() );
    arguments.getApplicationUsage()->setDescription( arguments.getApplicationName() + " visualizes the OSG bounding volume of the loaded model(s)." );
    arguments.getApplicationUsage()->setCommandLineUsage( arguments.getApplicationName() + " [options] filename ..." );

    arguments.getApplicationUsage()->addCommandLineOption( "--box", "Display the bounding box." );
    arguments.getApplicationUsage()->addCommandLineOption( "--sphere", "Display the bounding sphere. This is the default." );
    arguments.getApplicationUsage()->addCommandLineOption( "--both", "Display both the bounding sphere and bounding box." );
    arguments.getApplicationUsage()->addCommandLineOption( "--origin", "Render the model's origin with a circle and crosshair." );
    arguments.getApplicationUsage()->addCommandLineOption( "-v/--version", "Display the osgWorks version string." );

    if( arguments.read( "-h" ) || arguments.read( "--help" ) )
    {
        osg::notify( osg::ALWAYS ) << arguments.getApplicationUsage()->getDescription() << std::endl;
        arguments.getApplicationUsage()->write( osg::notify( osg::ALWAYS ), osg::ApplicationUsage::COMMAND_LINE_OPTION );
        return 1;
    }

    bool doBox( false );
    bool doSphere( true );
    if( arguments.find( "--box" ) > 0 )
    {
        doBox = true;
        doSphere = false;
    }
    if( arguments.find( "--both" ) > 0 )
        doBox = doSphere = true;
    if( arguments.find( "--sphere" ) > 0 )
        doSphere = true;
    
    const bool displayOrigin( arguments.find( "--origin" ) > 0 );

    if( arguments.read( "-v" ) || arguments.read( "--version" ) )
    {
        osg::notify( osg::ALWAYS ) << osgwTools::getVersionString() << std::endl << std::endl;
    }

    osg::ref_ptr< osg::Group > root = new osg::Group;

    osg::ref_ptr< osg::Node > model = osgDB::readNodeFiles( arguments );
    if( model.get() == NULL )
    {
        osg::notify( osg::FATAL ) << "Unable to load model(S)." << std::endl;
        arguments.getApplicationUsage()->write( osg::notify( osg::FATAL ), osg::ApplicationUsage::COMMAND_LINE_OPTION );
        return( 1 );
    }
    root->addChild( model.get() );


    osg::ref_ptr< osg::Group > decorations = new osg::Group;
    root->addChild( decorations.get() );
    {
        osg::StateSet* ss = decorations->getOrCreateStateSet();
        ss->setMode( GL_LINE_SMOOTH, osg::StateAttribute::ON );
        ss->setAttributeAndModes( new osg::BlendFunc );

        ss->setAttributeAndModes( new osg::PolygonMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE ) );
        ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    }

    osg::ref_ptr< osg::MatrixTransform > mt;
    osg::ref_ptr< osg::Geode > geode;
    if( doSphere || doBox )
    {
        mt = new osg::MatrixTransform;
        decorations->addChild( mt.get() );
        geode = new osg::Geode;
        mt->addChild( geode.get() );
    }
    if( doSphere )
    {
        const osg::BoundingSphere bs( model->getBound() );
        mt->setMatrix( osg::Matrix::translate( bs._center ) );
        geode->addDrawable( osgwTools::makeGeodesicSphere( bs._radius, 1 ) );

        osg::notify( osg::ALWAYS ) << "Sphere:" << std::endl;
        osg::notify( osg::ALWAYS ) << "\tCenter\t" << bs._center << std::endl;
        osg::notify( osg::ALWAYS ) << "\tRadius\t" << bs._radius << std::endl;
    }
    if( doBox )
    {
        osg::ComputeBoundsVisitor cbv;
        model->accept( cbv );
        const osg::BoundingBox bb( cbv.getBoundingBox() );

        mt->setMatrix( osg::Matrix::translate( bb.center() ) );
        osg::Vec3 ext( bb._max - bb._min );
        geode->addDrawable( osgwTools::makeWireBox( ext * 0.5 ) );

        osg::notify( osg::ALWAYS ) << "Box:" << std::endl;
        osg::notify( osg::ALWAYS ) << "\tCenter\t" << bb.center() << std::endl;
        osg::notify( osg::ALWAYS ) << "\tRadius\t" << bb.radius() << std::endl;
        osg::notify( osg::ALWAYS ) << "\tExtents\t" << ext << std::endl;
    }

    if( displayOrigin )
        decorations->addChild( makeOrigin() );


    osgViewer::Viewer viewer( arguments );

    // add the window size toggle handler
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);
    
    viewer.setSceneData( root.get() );
    return( viewer.run() );
}



/** \page osgwbvv The osgwbvv Application
osgwbvv visualizes OSG bounding volumes as a box, a sphere, or both, and displays the
bounding extents on the console.

\section su Simple Usage
Try running osgwbvv on \c teapot.osg, one of the osgWorks data files:

\code
C:\Projects>osgwbvv teapot.osg
Sphere:
        Center  0.0542275 0 0.01875
        Radius  1.02558
\endcode

In addition to displaying the bounding sphere extents on the console, osgwbvv visualizes
the bounding sphere in an OSG window:
\image html osgwbvv00.jpg

\section clp Command Line Parameters
<table border="0">
  <tr>
    <td><b>--sphere</b></td>
    <td>Display the bounding sphere. This is the default.</td>
  </tr>
  <tr>
    <td><b>--box</b></td>
    <td>Display the bounding box.</td>
  </tr>
  <tr>
    <td><b>--both</b></td>
    <td>Display both the bounding sphere and bounding box.</td>
  </tr>
  <tr>
    <td><b>--origin</b></td>
    <td>Render the model's origin with a circle and crosshair.</td>
  </tr>
  <tr>
    <td><b>-v/--version</b></td>
    <td>Display the osgWorks version string.</td>
  </tr>
</table>

*/
