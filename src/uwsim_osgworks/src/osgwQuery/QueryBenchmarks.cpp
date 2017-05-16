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

#include <osgwQuery/QueryBenchmarks.h>
#include <osgwQuery/QueryAPI.h>
#include <osg/buffered_value>
#include <osg/ref_ptr>
#include <osg/Geometry>
#include <osg/Timer>
#include <osg/GL>


namespace osgwQuery
{


static osg::buffered_value< osg::ref_ptr< QueryBenchmarks > > s_query_benchmarks;

QueryBenchmarks* getQueryBenchmarks( unsigned int contextID, osg::RenderInfo* ri )
{
    if( !( s_query_benchmarks[ contextID ] ) )
        s_query_benchmarks[ contextID ] = new QueryBenchmarks( contextID, ri );
    return( s_query_benchmarks[ contextID ].get() );
}


QueryBenchmarks::QueryBenchmarks( unsigned int contextID, osg::RenderInfo* ri )
  : osg::Referenced(),
    _trSetup( 0. ), _trTriangle( 0. ), _trFragment( 0. ),
    _toSetup( 0. ), _toFragment( 0. ),
    _toLatency( 0. ), _toOverhead( 0. ),
    _initialized( false )

{
    internalInit( contextID, ri );
}
QueryBenchmarks::~QueryBenchmarks()
{
}

void QueryBenchmarks::internalInit( unsigned int contextID, osg::RenderInfo* ri )
{
    double width, height;
    if( ( ri != NULL ) && ( ri->getCurrentCamera() != NULL ) )
    {
        const osg::Viewport* vp = ri->getCurrentCamera()->getViewport();
        width = vp->width();
        height = vp->height();
    }
    else
    {
        width = 1024.;
        height = 768.;
    }

    // This is the only state setup we do:
    glMatrixMode( GL_PROJECTION );
    glLoadMatrix( osg::Matrixf::ortho( 0., width, 0., height, -1., 1. ).ptr() );
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();

    osg::ref_ptr< osg::Geometry > geom = new osg::Geometry;
    osg::Vec3Array* v = new osg::Vec3Array;
    osg::DrawElementsUInt* deui = new osg::DrawElementsUInt( GL_TRIANGLE_STRIP, width * 2. );

    geom->setUseDisplayList( false );
    geom->setUseVertexBufferObjects( true );
    geom->setVertexArray( v );
    geom->addPrimitiveSet( deui );

    //
    // Measure setup time for normal rendering.

    unsigned int numVerts = 4;
    v->resize( numVerts );
    osg::Vec3* vert = &( (*v)[0] );
    unsigned int* index = &( (*deui)[0] );
    unsigned int idx;
    for( idx=0; idx<(numVerts / 2); idx++ )
    {
        vert->set( idx, 3., 0. );
        vert++;
        *index++ = idx * 2;
        vert->set( idx, 1., 0. );
        vert++;
        *index++ = idx * 2 + 1;
    }


    time( geom.get(), *ri );
    double t = time( geom.get(), *ri );
    _trSetup = t;

    //
    // Measure setup time for occlusion queries.

    osgwQuery::QueryAPI* qapi = osgwQuery::getQueryAPI( contextID );

    time( geom.get(), *ri, qapi );
    t = time( geom.get(), *ri, qapi );
    _toSetup = t;


    //
    // Measure triangle time for normal rendering.

    geom = new osg::Geometry;
    v = new osg::Vec3Array;
    deui = new osg::DrawElementsUInt( GL_TRIANGLE_STRIP, width * 2. );

    geom->setUseDisplayList( false );
    geom->setUseVertexBufferObjects( true );
    geom->setVertexArray( v );
    geom->addPrimitiveSet( deui );

    numVerts = (unsigned int)( width ) * 2;
    v->resize( numVerts );
    vert = &( (*v)[0] );
    index = &( (*deui)[0] );
    for( idx=0; idx<(numVerts / 2); idx++ )
    {
        vert->set( idx, 3., 0. );
        vert++;
        *index++ = idx * 2;
        vert->set( idx, 1., 0. );
        vert++;
        *index++ = idx * 2 + 1;
    }

    time( geom.get(), *ri );
    t = time( geom.get(), *ri );
    if( t < _trSetup )
    {
        osg::notify( osg::WARN ) << "QueryBenchmarks: Suspicious timing result for _trTriangle." << std::endl;
        _trTriangle = ( _trSetup ) / (double)( numVerts - 2 );
    }
    else
        _trTriangle = ( t - _trSetup ) / (double)( numVerts - 2 );


    //
    // Measure fragment time for normal rendering.

    geom = new osg::Geometry;
    v = new osg::Vec3Array;
    deui = new osg::DrawElementsUInt( GL_TRIANGLE_STRIP, width * 2. );

    geom->setUseDisplayList( false );
    geom->setUseVertexBufferObjects( true );
    geom->setVertexArray( v );
    geom->addPrimitiveSet( deui );

    numVerts = 4;
    v->resize( numVerts );
    vert = &( (*v)[0] );
    index = &( (*deui)[0] );
    {
        vert->set( 0., 0., 0. );
        vert++;
        *index++ = 0;
        vert->set( width, 0., 0. );
        vert++;
        *index++ = 1;
        vert->set( 0., height, 0. );
        vert++;
        *index++ = 2;
        vert->set( width, height, 0. );
        vert++;
        *index++ = 3;
    }

    time( geom.get(), *ri );
    t = time( geom.get(), *ri );
    if( t < _trSetup )
    {
        osg::notify( osg::WARN ) << "QueryBenchmarks: Suspicious timing result for _trFragment." << std::endl;
        _trFragment = ( _trSetup ) / ( width * height );
    }
    else
        _trFragment = ( t - _trSetup ) / ( width * height );

    //
    // Measure fragment time for occlusion queries.

    time( geom.get(), *ri, qapi );
    t = time( geom.get(), *ri, qapi );
    if( t < _toSetup )
    {
        osg::notify( osg::WARN ) << "QueryBenchmarks: Suspicious timing result for _toFragment." << std::endl;
        _toFragment = ( _toSetup ) / ( width * height );
    }
    else
        _toFragment = ( t - _toSetup ) / ( width * height );


    // TBD
    _toLatency = 1.; // Used to determine if Group children should be inserted. Not possible in OSG.
    _toOverhead = 1.; // Guthe doesn't appear to describe how to measure this, or how it is used in his algorithm.
}

#define LOOPCOUNT 100
#define DRAWCOUNT ((double)LOOPCOUNT * 5. )
#define DRAWCOUNT_INT (LOOPCOUNT * 5 )

double QueryBenchmarks::time( osg::Drawable* draw, osg::RenderInfo& ri )
{
    glFinish();

    int idx( LOOPCOUNT );
    osg::Timer timer;
    timer.setStartTick();
    while( idx-- > 0 )
    {
        draw->drawImplementation( ri );
        draw->drawImplementation( ri );
        draw->drawImplementation( ri );
        draw->drawImplementation( ri );
        draw->drawImplementation( ri );
    }

    glFlush();
    const double t = timer.time_s();
    return( t / DRAWCOUNT );
}

double QueryBenchmarks::time( osg::Drawable* draw, osg::RenderInfo& ri, osgwQuery::QueryAPI* qapi )
{
    GLuint ids[ DRAWCOUNT_INT ];
    qapi->glGenQueries( DRAWCOUNT_INT, ids );
    GLuint* idPtr = &( ids[ 0 ] );

    glFinish();

    int idx( LOOPCOUNT );
    osg::Timer timer;
    timer.setStartTick();
    while( idx-- > 0 )
    {
        qapi->glBeginQuery( GL_SAMPLES_PASSED, *idPtr++ );
        draw->drawImplementation( ri );
        qapi->glEndQuery( GL_SAMPLES_PASSED );
        qapi->glBeginQuery( GL_SAMPLES_PASSED, *idPtr++ );
        draw->drawImplementation( ri );
        qapi->glEndQuery( GL_SAMPLES_PASSED );
        qapi->glBeginQuery( GL_SAMPLES_PASSED, *idPtr++ );
        draw->drawImplementation( ri );
        qapi->glEndQuery( GL_SAMPLES_PASSED );
        qapi->glBeginQuery( GL_SAMPLES_PASSED, *idPtr++ );
        draw->drawImplementation( ri );
        qapi->glEndQuery( GL_SAMPLES_PASSED );
        qapi->glBeginQuery( GL_SAMPLES_PASSED, *idPtr++ );
        draw->drawImplementation( ri );
        qapi->glEndQuery( GL_SAMPLES_PASSED );
    }

    glFlush();
    const double t = timer.time_s();

    qapi->glDeleteQueries( DRAWCOUNT_INT, ids );

    return( t / DRAWCOUNT );
}



void InitCallback::operator()( osg::RenderInfo& renderInfo ) const
{
    if( _initialized )
        return;
    unsigned int contextID = renderInfo.getState()->getContextID();
    osgwQuery::QueryBenchmarks* qb = osgwQuery::getQueryBenchmarks( contextID, &renderInfo );
    _initialized = true;
}


// osgwQuery
}
