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

#include <osgwTools/AbsoluteModelTransform.h>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgViewer/Viewer>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/Geode>
#include <osg/Geometry>

#include <string>
#include <osg/io_utils>


osg::Node*
makeScene()
{
    osg::Group* root = new osg::Group;

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable(
        osg::createTexturedQuadGeometry( osg::Vec3( -2., 0., -2. ),
            osg::Vec3( 4., 0., 0. ), osg::Vec3( 0., 0., 4. ) ) );

    osgwTools::AbsoluteModelTransform* amt = new
        osgwTools::AbsoluteModelTransform( osg::Matrix::translate( osg::Vec3( 0, 0, -6 ) ) );
    //amt->setReferenceFrame( osg::Transform::RELATIVE_RF );
    amt->addChild( geode );

    osg::Matrix m(
        osg::Matrix::translate( osg::Vec3( 5, -3, 0 ) )
        );
    osg::MatrixTransform* mt = new osg::MatrixTransform( m );
    mt->addChild( amt );
    root->addChild( mt );

    return( (osg::Node*) root );
}


int
main( int argc,
      char ** argv )
{
    osg::ref_ptr< osg::Group > root = new osg::Group;
    root->addChild( makeScene() );
    osgDB::writeNodeFile( *root, "testamt.osg" );

    root->addChild( osgDB::readNodeFile( "cow.osg" ) );


    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 10, 30, 800, 600 );
    viewer.setSceneData( root.get() );

    return( viewer.run() );
}

