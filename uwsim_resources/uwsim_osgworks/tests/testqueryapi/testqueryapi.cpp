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
#include <osgViewer/Viewer>
#include <osg/Camera>
#include <osg/Notify>

#include <string>


/* \cond */
class PostDraw : public osg::Camera::DrawCallback
{
public:
    PostDraw( const std::string& name )
      : _qapi( NULL ),
        _name( name )
    {}

    virtual void operator()( osg::RenderInfo& renderInfo ) const
    {
        const osg::State* state = renderInfo.getState();
        const unsigned int contextID = state->getContextID();

        osg::setNotifyLevel( osg::INFO );
        _qapi = osgwQuery::getQueryAPI( contextID );
        osg::setNotifyLevel( osg::NOTICE );
    }

    // Return 0 for pass, 1 for fail.
    int success()
    {
        if( _qapi != NULL )
            osg::notify( osg::INFO ) << _name << ": PASSED." << std::endl;
        return( ( _qapi==NULL ) ? 1 : 0 );
    }

protected:
    mutable osgwQuery::QueryAPI* _qapi;
    std::string _name;
};
/* \endcond */


int main( int argc, char ** argv )
{
    osg::notify( osg::ALWAYS ) <<
        "This is a CTest regression test. To launch under Visual Studio, build the" << std::endl <<
        "RUN_TESTS target. Under Linux, enter 'make test' at a shell prompty." << std::endl <<
        std::endl;

    osg::ref_ptr< PostDraw > pd = new PostDraw( std::string( argv[ 0 ] ) );

    osgViewer::Viewer viewer;
    viewer.setThreadingModel( osgViewer::ViewerBase::SingleThreaded );
    viewer.setUpViewInWindow( 0, 0, 100, 100 );
    viewer.getCamera()->setPostDrawCallback( pd.get() );
    viewer.realize();

    // Draw an arbitrary 3 frames.
    for( int idx=0; idx<3; idx++ )
        viewer.frame();

    return( pd->success() );
}
