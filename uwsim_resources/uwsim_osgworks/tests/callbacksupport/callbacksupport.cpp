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
// This test verifies CommpositeDrawCallback functionality by attaching
// multiple post draw callbacks to a Camera node.
//

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/Camera>

#include <osgwTools/CallbackSupport.h>


/* \cond */

struct MyCallback : public osg::Camera::DrawCallback
{
public:
    MyCallback( unsigned int num )
      : _num( num ) {}

    virtual void operator()( osg::RenderInfo& renderInfo ) const
    {
        osg::notify( osg::ALWAYS ) << " Executing callback " << _num << std::endl;
    }

protected:
    unsigned int _num;
};

/** \endcond */


int
main( int argc, char ** argv )
{
    osgViewer::Viewer viewer;
    viewer.setSceneData( osgDB::readNodeFile( "teapot.osg" ) );
    if( viewer.getSceneData() == NULL )
        viewer.setSceneData( osgDB::readNodeFile( "cow.osg" ) );
    if( viewer.getSceneData() == NULL )
    {
        osg::notify( osg::FATAL ) << "Can't load teapot.osg or cow.osg." << std::endl;
        return( 1 );
    }

    osg::ref_ptr< osgwTools::CompositeDrawCallback > cdc = new osgwTools::CompositeDrawCallback;
    cdc->getDrawCallbackList().push_back( new MyCallback( 0 ) );
    cdc->getDrawCallbackList().push_back( new MyCallback( 1 ) );
    cdc->getDrawCallbackList().push_back( new MyCallback( 2 ) );
    viewer.getCamera()->setPostDrawCallback( cdc.get() );

    return( viewer.run() );
}

