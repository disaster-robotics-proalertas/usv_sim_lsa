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
#include <osg/Camera>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osgwTools/Version.h>
#include <osgwTools/Shapes.h>

#include <string>


const int winW( 800 ), winH( 600 );


osg::Node*
postRender( osgViewer::Viewer& viewer )
{
    osg::Camera* rootCamera( viewer.getCamera() );

    // Create the texture; we'll use this as our color buffer.
    // Note it has no image data; not required.
    osg::Texture2D* tex = new osg::Texture2D;
    tex->setTextureWidth( winW );
    tex->setTextureHeight( winH );
    tex->setInternalFormat( GL_RGBA );
    tex->setBorderWidth( 0 );
    tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::NEAREST );
    tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::NEAREST );

    // Attach the texture to the camera. Tell it to use multisampling.
    // Internally, OSG allocates a multisampled renderbuffer, renders to it,
    // and at the end of the frame performs a BlitFramebuffer into our texture.
    rootCamera->attach( osg::Camera::COLOR_BUFFER0, tex, 0, 0, false, 8, 8 );
    rootCamera->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT, osg::Camera::FRAME_BUFFER );
#if( OSGWORKS_OSG_VERSION >= 20906 )
    rootCamera->setImplicitBufferAttachmentMask(
        osg::Camera::IMPLICIT_COLOR_BUFFER_ATTACHMENT|osg::Camera::IMPLICIT_DEPTH_BUFFER_ATTACHMENT,
        osg::Camera::IMPLICIT_COLOR_BUFFER_ATTACHMENT );
#endif


    // Configure postRenderCamera to draw fullscreen textured quad
    osg::ref_ptr< osg::Camera > postRenderCamera( new osg::Camera );
    postRenderCamera->setClearMask( 0 );
    postRenderCamera->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER, osg::Camera::FRAME_BUFFER );

    postRenderCamera->setReferenceFrame( osg::Camera::ABSOLUTE_RF );
    postRenderCamera->setRenderOrder( osg::Camera::POST_RENDER );
    postRenderCamera->setViewMatrix( osg::Matrixd::identity() );
    postRenderCamera->setProjectionMatrix( osg::Matrixd::identity() );

    osg::Geode* geode( new osg::Geode );
    geode->addDrawable( osgwTools::makePlane(
        osg::Vec3( -1,-1,0 ), osg::Vec3( 2,0,0 ), osg::Vec3( 0,2,0 ) ) );
    geode->getOrCreateStateSet()->setTextureAttributeAndModes(
        0, tex, osg::StateAttribute::ON );
    geode->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    geode->getOrCreateStateSet()->setMode( GL_DEPTH_TEST, osg::StateAttribute::OFF );

    postRenderCamera->addChild( geode );

    return( postRenderCamera.release() );
}

int
main( int argc, char** argv )
{
    osg::notify( osg::ALWAYS ) <<
// cols:  12345678901234567890123456789012345678901234567890123456789012345678901234567890
         "This is an example of doing render to texture in OSG, with the result displayed" << std::endl <<
         "on a fullscreen triangle pair. It uses only one osgWorks OSG version feature to" << std::endl <<
         "configure the destination textures and RTT Cameras appropriately, and uses the" << std::endl <<
         "plane shape <osgwTools/Shapes.h> for the final texture display." << std::endl;

    osg::ArgumentParser arguments( &argc, argv );
    osg::ref_ptr< osg::Group > root( new osg::Group );
    root->addChild( osgDB::readNodeFiles( arguments ) );
    if( root->getNumChildren() == 0 )
    {
        // Load default cow model.
        std::string fileName( "cow.osg" );
        root->addChild( osgDB::readNodeFile( fileName ) );
    }
    if( root->getNumChildren() == 0 )
        return( 1 );

    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 10, 30, winW, winH );
    viewer.setSceneData( root.get() );
    viewer.realize();

    root->addChild( postRender( viewer ) );

    // Clear to white to make AA extremely obvious.
    viewer.getCamera()->setClearColor( osg::Vec4( 1., 1., 1., 1. ) );

    return( viewer.run() );
}



/** \page rtt The rtt Example
rtt demonstrates rendering to texture and displaying that texture, with a
minimum amount of code (less than 100 lines) and full osgGA camera manipulator
support.

This is a pure-OSG example that doesn't demonstrate any osgWorks-specific
functionalitty. However, it demonstrates rendering to texture, then displaying
that texture on a full-viewport quad, the basis of most modern GPU-based
rendering. Furthermore, it demonstrates how to do this in a minimum amount of
code, so that it's easy to see how the scene graph is configured.
*/
