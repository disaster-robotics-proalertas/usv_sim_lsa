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
#include <osg/buffered_value>
#include <osg/ref_ptr>
#include <osg/GLExtensions>


namespace osgwQuery
{


static osg::buffered_value< osg::ref_ptr< QueryAPI > > s_query_api;

QueryAPI* getQueryAPI( unsigned int contextID )
{
    if( !( s_query_api[ contextID ] ) )
        s_query_api[ contextID ] = new QueryAPI( contextID );
    return( s_query_api[ contextID ].get() );
}


QueryAPI::QueryAPI( unsigned int contextID )
  : osg::Referenced(),
    _initialized( false ),
    _queryAPISupported( false ),
    _timerQuerySupported( false ),
    _transformFeedback3Supported( false ),
    _transformFeedbackSupported( false ),
    _occlusionQuery2Supported( false )
{
    internalInit( contextID );
}
QueryAPI::~QueryAPI()
{
}

void QueryAPI::internalInit( unsigned int contextID )
{
    _queryAPISupported = osg::isGLExtensionOrVersionSupported( contextID, "GL_ARB_occlusion_query", 1.5f );
    if( _queryAPISupported )
    {
        _beginQuery = (BeginQueryProc)( osg::getGLExtensionFuncPtr( "glBeginQuery", "glBeginQueryARB" ) );
        _endQuery = (EndQueryProc)( osg::getGLExtensionFuncPtr( "glEndQuery", "glEndQueryARB" ) );
        _genQueries = (GenQueriesProc)( osg::getGLExtensionFuncPtr( "glGenQueries", "glGenQueriesARB" ) );
        _deleteQueries = (DeleteQueriesProc)( osg::getGLExtensionFuncPtr( "glDeleteQueries", "glDeleteQueriesARB" ) );
        _isQuery = (IsQueryProc)( osg::getGLExtensionFuncPtr( "glIsQuery", "glIsQueryARB" ) );
        _getQueryiv = (GetQueryivProc)( osg::getGLExtensionFuncPtr( "glGetQueryiv", "glGetQueryivARB" ) );
        _getQueryObjectiv = (GetQueryObjectivProc)( osg::getGLExtensionFuncPtr( "glGetQueryObjectiv", "glGetQueryObjectivARB" ) );
        _getQueryObjectuiv = (GetQueryObjectuivProc)( osg::getGLExtensionFuncPtr( "glGetQueryObjectuiv", "glGetQueryObjectuivARB" ) );
    }

    _timerQuerySupported = osg::isGLExtensionOrVersionSupported( contextID, "GL_ARB_timer_query", 3.3f );
    if( _timerQuerySupported )
    {
        _getQueryObjecti64v = (GetQueryObjecti64vProc)( osg::getGLExtensionFuncPtr( "glGetQueryObjecti64v", "glGetQueryObjecti64vARB" ) );
        _getQueryObjectui64v = (GetQueryObjectui64vProc)( osg::getGLExtensionFuncPtr( "glGetQueryObjectui64v", "glGetQueryObjectui64vARB" ) );
    }

    _transformFeedback3Supported = osg::isGLExtensionOrVersionSupported( contextID, "GL_ARB_transform_feedback3", 4.0f );
    if( _transformFeedback3Supported )
    {
        _beginQueryIndexed = (BeginQueryIndexedProc)( osg::getGLExtensionFuncPtr( "glBeginQueryIndexed", "glBeginQueryIndexedARB" ) );
        _endQueryIndexed = (EndQueryIndexedProc)( osg::getGLExtensionFuncPtr( "glEndQueryIndexed", "glEndQueryIndexedARB" ) );
        _getQueryIndexediv = (GetQueryIndexedivProc)( osg::getGLExtensionFuncPtr( "glGetQueryIndexediv", "glGetQueryIndexedivARB" ) );
    }

    _transformFeedbackSupported = osg::isGLExtensionOrVersionSupported( contextID, "GL_EXT_transform_feedback", 3.0f );
    _occlusionQuery2Supported = osg::isGLExtensionOrVersionSupported( contextID, "GL_ARB_occlusion_query2", 3.3f );

    osg::notify( osg::INFO ) << "osgwQuery: Query API availability:" << std::endl;
    osg::notify( osg::INFO ) << "\tBase query API: " << std::boolalpha << _queryAPISupported << std::endl;
    osg::notify( osg::INFO ) << "\tOcc Query 2: " << std::boolalpha << _occlusionQuery2Supported << std::endl;
    osg::notify( osg::INFO ) << "\tTimer query: " << std::boolalpha << _timerQuerySupported << std::endl;
    osg::notify( osg::INFO ) << "\tXform feedback: " << std::boolalpha << _transformFeedbackSupported << std::endl;
    osg::notify( osg::INFO ) << "\tXform feedback 3: " << std::boolalpha << _transformFeedback3Supported << std::endl;

    _initialized = true;
}


void QueryAPI::glBeginQuery( GLenum target, GLuint id ) const
{
#ifdef _DEBUG
    if( !_initialized || ( _beginQuery == NULL ) )
        osg::notify( osg::WARN ) << "osgwQuerl::QueryAPI: glBeginQuery NULL" << std::endl;
#endif
    _beginQuery( target, id );
}
void QueryAPI::glEndQuery( GLenum target ) const
{
#ifdef _DEBUG
    if( !_initialized || ( _endQuery == NULL ) )
        osg::notify( osg::WARN ) << "osgwQuerl::QueryAPI: glEndQuery NULL" << std::endl;
#endif
    _endQuery( target );
}
void QueryAPI::glGenQueries( GLsizei n, GLuint *ids ) const
{
#ifdef _DEBUG
    if( !_initialized || ( _genQueries == NULL ) )
        osg::notify( osg::WARN ) << "osgwQuerl::QueryAPI: glGenQueries NULL" << std::endl;
#endif
    _genQueries( n, ids );
}
void QueryAPI::glDeleteQueries( GLsizei n, const GLuint *ids ) const
{
#ifdef _DEBUG
    if( !_initialized || ( _deleteQueries == NULL ) )
        osg::notify( osg::WARN ) << "osgwQuerl::QueryAPI: glDeleteQueries NULL" << std::endl;
#endif
    _deleteQueries( n, ids );
}
GLboolean QueryAPI::glIsQuery( GLuint id ) const
{
#ifdef _DEBUG
    if( !_initialized || ( _isQuery == NULL ) )
        osg::notify( osg::WARN ) << "osgwQuerl::QueryAPI: glIsQuery NULL" << std::endl;
#endif
    return( _isQuery( id ) );
}
void QueryAPI::glGetQueryiv( GLenum target, GLenum pname, int *params ) const
{
#ifdef _DEBUG
    if( !_initialized || ( _getQueryiv == NULL ) )
        osg::notify( osg::WARN ) << "osgwQuerl::QueryAPI: glGetQueryiv NULL" << std::endl;
#endif
    _getQueryiv( target, pname, params );
}
void QueryAPI::glGetQueryObjectiv( GLuint id, GLenum pname, GLint *params ) const
{
#ifdef _DEBUG
    if( !_initialized || ( _getQueryObjectiv == NULL ) )
        osg::notify( osg::WARN ) << "osgwQuerl::QueryAPI: glGetQueryObjectiv NULL" << std::endl;
#endif
    _getQueryObjectiv( id, pname, params );
}
void QueryAPI::glGetQueryObjectuiv( GLuint id, GLenum pname, GLuint *params ) const
{
#ifdef _DEBUG
    if( !_initialized || ( _getQueryObjectuiv == NULL ) )
        osg::notify( osg::WARN ) << "osgwQuerl::QueryAPI: glGetQueryObjectuiv NULL" << std::endl;
#endif
    _getQueryObjectuiv( id, pname, params );
}
void QueryAPI::glGetQueryObjecti64v( GLuint id, GLenum pname, GLint64EXT *params ) const
{
#ifdef _DEBUG
    if( !_initialized || ( _getQueryObjecti64v == NULL ) )
        osg::notify( osg::WARN ) << "osgwQuerl::QueryAPI: glGetQueryObjecti64v NULL" << std::endl;
#endif
    _getQueryObjecti64v( id, pname, params );
}
void QueryAPI::glGetQueryObjectui64v( GLuint id, GLenum pname, GLuint64EXT *params ) const
{
#ifdef _DEBUG
    if( !_initialized || ( _getQueryObjectui64v == NULL ) )
        osg::notify( osg::WARN ) << "osgwQuerl::QueryAPI: glGetQueryObjectui64v NULL" << std::endl;
#endif
    _getQueryObjectui64v( id, pname, params );
}
void QueryAPI::glBeginQueryIndexed( GLenum target, GLuint index, GLuint id ) const
{
#ifdef _DEBUG
    if( !_initialized || ( _beginQueryIndexed == NULL ) )
        osg::notify( osg::WARN ) << "osgwQuerl::QueryAPI: glBeginQueryIndexed NULL" << std::endl;
#endif
    _beginQueryIndexed( target, index, id );
}
void QueryAPI::glEndQueryIndexed( GLenum target, GLuint index ) const
{
#ifdef _DEBUG
    if( !_initialized || ( _endQueryIndexed == NULL ) )
        osg::notify( osg::WARN ) << "osgwQuerl::QueryAPI: glEndQueryIndexed NULL" << std::endl;
#endif
    _endQueryIndexed( target, index );
}
void QueryAPI::glGetQueryIndexediv( GLenum target, GLuint index, GLenum pname, GLint *params ) const
{
#ifdef _DEBUG
    if( !_initialized || ( _getQueryIndexediv == NULL ) )
        osg::notify( osg::WARN ) << "osgwQuerl::QueryAPI: glGetQueryIndexediv NULL" << std::endl;
#endif
    _getQueryIndexediv( target, index, pname, params );
}


// osgwQuery
}
