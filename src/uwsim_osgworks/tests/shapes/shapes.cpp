/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgWorks is (C) Copyright 2010 by Kenneth Mark Bryden
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

#include <osgViewer/Viewer>
#include <osgGA/StateSetManipulator>
#include <osgDB/ReadFile>

#include <osg/MatrixTransform>
#include <osg/Texture2D>

#include <osgwTools/Shapes.h>

int
main( int argc,
      char ** argv )
{
    osg::ref_ptr< osg::Group > root = new osg::Group;

    osg::ref_ptr< osg::Geode > geode;
    osg::ref_ptr< osg::MatrixTransform > mt;


    // Objects in foreground are transformed by parent Transform node.

    geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeGeodesicSphere( 1., 2 ) );
    mt = new osg::MatrixTransform( osg::Matrix::translate( -3., 0., 3. ) );
    root->addChild( mt.get() );
    mt->addChild( geode.get() );

    geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeAltAzSphere( 1., 8, 16 ) );
    mt = new osg::MatrixTransform( osg::Matrix::translate( 0., 0., 3. ) );
    root->addChild( mt.get() );
    mt->addChild( geode.get() );
    
    geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeWireAltAzSphere( 1., 5, 7 ) );
    mt = new osg::MatrixTransform( osg::Matrix::translate( 3., 0., 3. ) );
    root->addChild( mt.get() );
    mt->addChild( geode.get() );

    geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeBox( osg::Vec3( .75, .75, .75 ), osg::Vec3s( 2, 2, 3 ) ) );
    mt = new osg::MatrixTransform( osg::Matrix::translate( -3., 0., 0. ) );
    root->addChild( mt.get() );
    mt->addChild( geode.get() );

    geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeWireBox( osg::Vec3( .75, .75, .75 ) ) );
    mt = new osg::MatrixTransform( osg::Matrix::translate( 0., 0., 0. ) );
    root->addChild( mt.get() );
    mt->addChild( geode.get() );

    geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeArrow() );
    mt = new osg::MatrixTransform( osg::Matrix::translate( -3., 0., -3. ) );
    root->addChild( mt.get() );
    mt->addChild( geode.get() );

    geode = new osg::Geode;
    geode->addDrawable( osgwTools::makePlane( osg::Vec3( -.75, 0.25, -1. ),
        osg::Vec3( 1.5, 0., 0. ), osg::Vec3( 0., -0.25, 2. ), osg::Vec2s( 2, 3) ) );
    mt = new osg::MatrixTransform( osg::Matrix::translate( 0., 0., -3. ) );
    root->addChild( mt.get() );
    mt->addChild( geode.get() );

    geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeWirePlane( osg::Vec3( -.75, -0.25, -1. ),
        osg::Vec3( 1.5, 0., 0. ), osg::Vec3( 0., 0.25, 2. ), osg::Vec2s( 2, 3) ) );
    mt = new osg::MatrixTransform( osg::Matrix::translate( 3., 0., -3. ) );
    root->addChild( mt.get() );
    mt->addChild( geode.get() );

    geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeWireCircle( 1., 7, osg::Vec3( 1., 0., 0. ) ) );
    mt = new osg::MatrixTransform( osg::Matrix::translate( 3., 0., 0. ) );
    root->addChild( mt.get() );
    mt->addChild( geode.get() );

    geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeCircle( 1., 32, osg::Vec3( 0., -1., 0. ) ) );
    mt = new osg::MatrixTransform( osg::Matrix::translate( 6., 0., 0. ) );
    root->addChild( mt.get() );
    mt->addChild( geode.get() );

    geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeCircle() );
    mt = new osg::MatrixTransform( osg::Matrix::translate( 6., 0., -3. ) );
    root->addChild( mt.get() );
    mt->addChild( geode.get() );

    geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeOpenCylinder( 1.5, .75, .5, osg::Vec2s( 2, 14 ) ) );
    mt = new osg::MatrixTransform( osg::Matrix::translate( 9., 0., -3. ) );
    root->addChild( mt.get() );
    mt->addChild( geode.get() );

    geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeClosedCylinder( 1.5, .6, .6, true, true, osg::Vec2s( 2, 14 ) ) );
    mt = new osg::MatrixTransform( osg::Matrix::translate( 9., 0., 0. ) );
    root->addChild( mt.get() );
    mt->addChild( geode.get() );

    geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeWireCylinder( 1.5, .75, .5, osg::Vec2s( 2, 14 ) ) );
    mt = new osg::MatrixTransform( osg::Matrix::translate( 9., 0., 3. ) );
    root->addChild( mt.get() );
    mt->addChild( geode.get() );

    geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeCone( 1.5, .75, osg::Vec2s( 3, 12 ) ) );
    mt = new osg::MatrixTransform( osg::Matrix::translate( 6., 0., 3. ) );
    root->addChild( mt.get() );
    mt->addChild( geode.get() );

    geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeWireCapsule( 1.9, .5, osg::Vec2s( 2, 12 ) ) );
    mt = new osg::MatrixTransform( osg::Matrix::translate( 6., 0., -6. ) );
    root->addChild( mt.get() );
    mt->addChild( geode.get() );

    geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeCapsule( 1.9, .5, osg::Vec2s( 2, 12 ) ) );
    mt = new osg::MatrixTransform( osg::Matrix::translate( 9., 0., -6. ) );
    root->addChild( mt.get() );
    mt->addChild( geode.get() );


    // Objects in background are transformed directly in Shapes generation code.

    osg::Vec3 axis( 1., 2., 3. );
    axis.normalize();

    geode = new osg::Geode;
    osg::Matrix m = osg::Matrix::rotate( .8, axis ) * osg::Matrix::translate( -3., 5., 0. );
    geode->addDrawable( osgwTools::makeBox( m, osg::Vec3( .75, .75, .75 ), osg::Vec3s( 2, 2, 2 ) ) );
    root->addChild( geode.get() );

    geode = new osg::Geode;
    m = osg::Matrix::rotate( .8, axis ) * osg::Matrix::translate( -3., 5., -3. );
    geode->addDrawable( osgwTools::makeArrow( m ) );
    root->addChild( geode.get() );

    geode = new osg::Geode;
    m = osg::Matrix::rotate( .8, axis ) * osg::Matrix::translate( 9., 5., -3. );
    geode->addDrawable( osgwTools::makeOpenCylinder( m, 1.5, .75, .5, osg::Vec2s( 2, 14 ) ) );
    root->addChild( geode.get() );


    geode = new osg::Geode;
    m = osg::Matrix::rotate( .8, axis ) * osg::Matrix::translate( 9., 5., 0. );
    geode->addDrawable( osgwTools::makeClosedCylinder( m, 1.5, .6, .6, true, true, osg::Vec2s( 2, 14 ) ) );
    root->addChild( geode.get() );


    if( false ) // Enable to test texture mapping and tex coords.
    {
        osg::Image* image = osgDB::readImageFile( "testpattern.png" );
        osg::Texture2D* tex = new osg::Texture2D( image );
        root->getOrCreateStateSet()->setTextureAttributeAndModes( 0, tex );
    }

    osgGA::StateSetManipulator* ssmanip = new osgGA::StateSetManipulator;
    ssmanip->setStateSet( root->getOrCreateStateSet() );

    osgViewer::Viewer viewer;
    viewer.addEventHandler( ssmanip );
    viewer.setSceneData( root.get() );
    return( viewer.run() );
}
