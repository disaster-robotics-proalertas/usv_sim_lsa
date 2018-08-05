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

#include <osgwQuery/QueryAPI.h>
#include <osgwQuery/QueryObject.h>
#include <osgwTools/Shapes.h>
#include <osgViewer/Viewer>
#include <osg/Drawable>
#include <osg/Notify>

#include <string>


/* \cond */
class DrawCallback : public osg::Drawable::DrawCallback
{
public:
    DrawCallback( const std::string& name )
      : _qapi( NULL ),
        _name( name ),
        _pixelCount( 0 ),
        _isQuery( GL_FALSE )
    {
        _qobj = new osgwQuery::QueryObject();
    }

    virtual void drawImplementation( osg::RenderInfo& renderInfo, const osg::Drawable* drawable ) const
    {
        const osg::State* state = renderInfo.getState();
        const unsigned int contextID = state->getContextID();

        if( _qapi == NULL )
        {
            osg::setNotifyLevel( osg::INFO );
            _qapi = osgwQuery::getQueryAPI( contextID );
            osg::setNotifyLevel( osg::NOTICE );
        }

        if( ( _qapi == NULL ) ||
            ( !( _qapi->getQueryAPISupported() ) ) )
        {
            // Nothing to test.
            drawable->drawImplementation( renderInfo );
            return;
        }

        GLuint id = _qobj->getID( contextID );
        _isQuery = _qapi->glIsQuery( id );

        _qapi->glBeginQuery( GL_SAMPLES_PASSED, id );

        drawable->drawImplementation( renderInfo );

        _qapi->glEndQuery( GL_SAMPLES_PASSED );

        _pixelCount = 0;
        _qapi->glGetQueryObjectuiv( id, GL_QUERY_RESULT, &_pixelCount );
    }

    // Return 0 for pass, 1 for fail.
    int success()
    {
        if( ( _qapi == NULL ) ||
            ( !( _qapi->getQueryAPISupported() ) ) ||
            ( _pixelCount == 0 ) ||
            ( _isQuery == GL_FALSE ) )
            return( 1 );

        osg::notify( osg::INFO ) << _name << ": PASSED." << std::endl;
        return( 0 );
    }

protected:
    mutable osgwQuery::QueryAPI* _qapi;
    mutable osg::ref_ptr< osgwQuery::QueryObject > _qobj;

    std::string _name;
    mutable GLuint _pixelCount;
    mutable GLboolean _isQuery;
};
/* \endcond */


int main( int argc, char ** argv )
{
    osg::notify( osg::ALWAYS ) <<
        "This is a CTest regression test. To launch under Visual Studio, build the" << std::endl <<
        "RUN_TESTS target. Under Linux, enter 'make test' at a shell prompty." << std::endl <<
        std::endl;

    osg::ref_ptr< DrawCallback > dcb = new DrawCallback( std::string( argv[ 0 ] ) );

    osgViewer::Viewer viewer;
    viewer.setThreadingModel( osgViewer::ViewerBase::SingleThreaded );
    viewer.setUpViewInWindow( 0, 0, 100, 100 );

    osg::Geometry* geom = osgwTools::makeGeodesicSphere();
    geom->setUseDisplayList( false );
    geom->setUseVertexBufferObjects( true );
    geom->setDrawCallback( dcb.get() );
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( geom );
    viewer.setSceneData( geode );

    viewer.getCamera()->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    viewer.getCamera()->setViewMatrix( osg::Matrix::identity() );
    viewer.getCamera()->setProjectionMatrix( osg::Matrix::ortho( -2., 2., -2., 2., -2., 2. ) );
    viewer.realize();

    // Draw an arbitrary 3 frames.
    for( int idx=0; idx<3; idx++ )
        viewer.frame();

    return( dcb->success() );
}
