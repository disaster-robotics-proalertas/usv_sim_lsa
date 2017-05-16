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

#include <osgwTools/Orientation.h>
#include <osgwTools/Shapes.h>

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgGA/GUIEventHandler>

#include <osg/MatrixTransform>
#include <osg/io_utils>
#include <osg/Notify>



class YPRManip : public osgGA::GUIEventHandler
{
public:
    YPRManip( osg::MatrixTransform* mt )
        : _mt( mt ),
          _ypr( 0., 0., 0. ),
          _orient( new osgwTools::Orientation() )
    {
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        bool handled( false );
        if( ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN )
        {
            double delta( 10. );
            switch( ea.getKey() )
            {
            case 'y': _ypr[0] += delta; handled = true;
                break;
            case 'Y': _ypr[0] -= delta; handled = true;
                break;
            case 'p': _ypr[1] += delta; handled = true;
                break;
            case 'P': _ypr[1] -= delta; handled = true;
                break;
            case 'r': _ypr[2] += delta; handled = true;
                break;
            case 'R': _ypr[2] -= delta; handled = true;
                break;
            default: break;
            }

            if( handled )
            {
                _mt->setMatrix( _orient->getMatrix( _ypr ) );
                OSG_ALWAYS << "Set: " << _ypr << ", from matrix: " << _orient->getYPR( _mt->getMatrix() ) << std::endl;
            }
        }
        return( handled );
    }

protected:
    ~YPRManip() {}

    osg::ref_ptr< osg::MatrixTransform > _mt;
    osg::Vec3d _ypr;

    osg::ref_ptr< osgwTools::Orientation > _orient;
};


int main( int argc, char** argv )
{
    osg::ref_ptr< osg::MatrixTransform > root( new osg::MatrixTransform );
    osg::ref_ptr< osg::Geode > geode( new osg::Geode );
    root->addChild( geode.get() );

    osg::Matrix m;
    osg::ref_ptr< osg::Vec4Array > c;
    osg::Geometry* geom;

    const osg::Vec3d baseUp( 0., 0., 1. );
    const osg::Vec3d baseRight( 1., 0., 0. );
    const osg::Vec3d baseDir( 0., 1., 0. );

    m.set( 1., 0., 0., 0.,
        0., 1., 0., 0.,
        baseUp[0], baseUp[1], baseUp[2], 0.,
        0., 0., 0., 1. );
    c = new osg::Vec4Array(); c->resize( 1 );
    (*c)[0].set( 0., 0., 1., 1. );
    geom = osgwTools::makeArrow( m );
    geom->setColorArray( c.get() );
    geode->addDrawable( geom );

    m.set( 0., 0., 1., 0.,
        0., 1., 0., 0.,
        baseRight[0], baseRight[1], baseRight[2], 0.,
        0., 0., 0., 1. );
    c = new osg::Vec4Array(); c->resize( 1 );
    (*c)[0].set( 1., 0., 0., 1. );
    geom = osgwTools::makeArrow( m );
    geom->setColorArray( c.get() );
    geode->addDrawable( geom );

    m.set( 1., 0., 0., 0.,
        0., 0., 1., 0.,
        baseDir[0], baseDir[1], baseDir[2], 0.,
        0., 0., 0., 1. );
    c = new osg::Vec4Array(); c->resize( 1 );
    (*c)[0].set( 0., 1., 0., 1. );
    geom = osgwTools::makeArrow( m );
    geom->setColorArray( c.get() );
    geode->addDrawable( geom );


    YPRManip* yprManip = new YPRManip( root.get() );

    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 20., 30., 1027., 768. );
    viewer.addEventHandler( yprManip );
    viewer.setSceneData( root.get() );
    return( viewer.run() );
}
