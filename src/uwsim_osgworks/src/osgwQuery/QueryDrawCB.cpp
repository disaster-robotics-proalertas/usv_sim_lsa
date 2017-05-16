/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgWorks is (C) Copyright 2009-2011 by Kenneth Mark Bryden
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

#include <osgwQuery/QueryDrawCB.h>
#include <osgwQuery/QueryAPI.h>
#include <osgwQuery/QueryBenchmarks.h>
#include <osgwTools/Shapes.h>
#include <osgwTools/CountsVisitor.h>
#include <OpenThreads/ScopedLock>
#include <osgUtil/CullVisitor>


namespace osgwQuery
{


double AddDrawCB::s_CscrOi( 0. );


QueryDrawCB::QueryDrawCB()
  : osg::Drawable::DrawCallback(),
    osg::Drawable::CullCallback(),
    _initialized( false ),
    _numVertices( 0 )
{
}
QueryDrawCB::QueryDrawCB( const QueryDrawCB& rhs, const osg::CopyOp& copyop )
  : osg::Drawable::DrawCallback( rhs, copyop ),
    osg::Drawable::CullCallback( rhs, copyop ),
    _initialized( false ),
    _numVertices( rhs._numVertices )
{
}

void QueryDrawCB::drawImplementation( osg::RenderInfo& renderInfo, const osg::Drawable* drawable ) const
{
    if( !_initialized )
    {
        OpenThreads::ScopedLock< OpenThreads::Mutex > lock( _lock );

        if( !( _queryDrawable.valid() ) )
            init( drawable->getBound() );
        _initialized = true;
    }


    const double cbbOiSqRt = 1. / ( _dOi * sqrt( _w / _h ) * 2. * tan( _thetaOver2 ) );
    const double cbbOi = cbbOiSqRt * cbbOiSqRt * _AbbOiOver6;

    // Compute pcovOi, the probability that this Drawable id covered.
    double pcovOi;
    const double cscrOi = AddDrawCB::getCscrOi();
    if( cbbOi < cscrOi )
    {
        double temp = sqrt( cscrOi ) - sqrt( cbbOi );
        pcovOi = temp * temp;
    }
    else
        pcovOi = 0.;

    // Compute pocclOi, the probability that the object is occluded,
    // accounting for temporal coherence. "p sub occl ( O sub i )" is
    // referred to as "p sub o ( O sub i )" after section 3.1, and
    // (in reference to a hierarchy) also as "p sub o ( H sub i )".
    // This is from personal communication with author Michael Guthe.
    double pocclOi;
    if( true /* previously visible */ )
        pocclOi = 0.5 * pcovOi * pcovOi;
    else if( false /* previously frustum culled */ )
        pocclOi = pcovOi;
    else /* previously occluded */
        pocclOi = 1.;



    unsigned int contextID = renderInfo.getState()->getContextID();
    osgwQuery::QueryBenchmarks* qb = osgwQuery::getQueryBenchmarks( contextID, &renderInfo );
    osgwQuery::QueryAPI* qapi = osgwQuery::getQueryAPI( contextID );
    QueryStatus& qs = _queries[ contextID ];

    /** Implements the Guthe paper'a "QueryReasonable" function, pseudocode
    in Guthe Figure 5, described in Guthe section 4. */
    bool queryReasonable( false );


    double cOi = _RcovOi * cbbOi;
    // TBD need max of triangle cost or fragment cost.
    const double pixels = _w * _h * cOi;
    const double fragRenderTime = pixels * qb->_trFragment + qb->_trSetup;
    // TBD QueryBenchmark should time per-vertex cost.
    // TBD vertRenderTime is a constant.
    const double vertRenderTime = _numVertices * qb->_trTriangle;
    const double renderTime = osg::maximum< double >( fragRenderTime, vertRenderTime );
    const double queryTime = pixels * qb->_toFragment + qb->_toSetup;

    if( ( renderTime < queryTime ) /* ||
            ( sum of child costs < node cost ) */ )
    {
        queryReasonable = false;
    }

    if( qs._queryActive )
    {
        GLint result;
        const GLuint id = qs._queryObject->getID( contextID );
        qapi->glGetQueryObjectiv( id, GL_QUERY_RESULT, &result );
        qs._wasOccluded = ( result < 25 ); // TBD need configurable threshold.
        qs._queryActive = false;
    }
    if( qs._wasOccluded )
    {
        queryReasonable = true;
        osg::notify( osg::ALWAYS ) << "Case 2: was occl, true" << std::endl;
    }

    /*
    if node cost > ( frames since node's last query * node benefit )
        return false;
        */
    /*
    for each child
        if child render time < child query time
            return true;
    if node benefit > sum of child benefits &&
            ( query overhead time * # of node leaves ) > node query time
        return true;
    */


    if( queryReasonable )
    {
        // Issue query.
        const GLuint id = qs._queryObject->getID( contextID );
        qapi->glBeginQuery( GL_SAMPLES_PASSED, id );
        _queryDrawable->drawImplementation( renderInfo );
        qapi->glEndQuery( GL_SAMPLES_PASSED );
        qs._queryActive = true;
    }
    else
    {
        // Process node.
        drawable->drawImplementation( renderInfo );
    }


#if 0
    // Increment CscrOi for next Drawable.
    AddDrawCB::setCscrOi( cscrOi + ( ( 1. - cscrOi ) * cbbOi ) );
#endif
}

bool QueryDrawCB::cull( osg::NodeVisitor* nv, osg::Drawable* drawable, osg::RenderInfo* renderInfo ) const
{
    osgUtil::CullVisitor* cv = static_cast< osgUtil::CullVisitor* >( nv );
    const osg::Matrix& view = *( cv->getModelViewMatrix() );
    const osg::Camera* cam = cv->getCurrentCamera();
    const osg::Viewport* vp = cam->getViewport();
    const osg::BoundingBox& bb = drawable->getBound();

    osg::Matrix proj = cam->getProjectionMatrix();
    double fovy, aspect, zNear, zFar;
    proj.getPerspective( fovy, aspect, zNear, zFar );

    {
        OpenThreads::ScopedLock< OpenThreads::Mutex > lock( _lock );

        _dOi = cv->getDistanceFromEyePoint( bb.center(), false );
        _w = vp->width();
        _h = vp->height();
        _thetaOver2 = osg::DegreesToRadians( fovy * .5 );
    }

    return( cv->isCulled( bb ) );
}

void QueryDrawCB::init( osg::BoundingBox bb ) const
{
    osg::Vec3 extents = bb._max - bb._min;


    //
    // Create box geometry for use as query geometry.

    osg::Vec3 halfExtents = extents / 2.;
    osg::Geometry* geom = osgwTools::makeBox( halfExtents );

    // Offset the query geometry box by the bounding box center.
    osg::Vec3Array* v = static_cast< osg::Vec3Array* >( geom->getVertexArray() );
    const osg::Vec3 center = bb.center();
    unsigned int idx;
    for( idx=0; idx<v->size(); idx++ )
        (*v)[ idx ] += center;

    // Optimize.
    geom->setUseDisplayList( false );
    geom->setUseVertexBufferObjects( true );
    geom->setTexCoordArray( 0, NULL );
    geom->setNormalArray( NULL );
    geom->setNormalBinding( osg::Geometry::BIND_OFF );

    _queryDrawable = geom;


    //
    // Compute and store constants defined in Guthe, section 3.1.

    // 1/6th of the bounding box surface area.
    const double abbOi = ( 2. * extents[ 0 ] * extents[ 1 ] ) +
        ( 2. * extents[ 1 ] * extents[ 2 ] ) +
        ( 2. * extents[ 2 ] * extents[ 0 ] );
    _AbbOiOver6 = abbOi / 6.;

    // Ratio of actual object screen area to boulding box screen area.
    const double aOi = 4. * osg::PI * bb.radius() * bb.radius();
    _RcovOi = 3. / 2. * aOi * abbOi;
}



QueryDrawCB::QueryStatus::QueryStatus()
  : _queryActive( false ),
    _wasOccluded( false )
{
}



void AddDrawCB::apply( osg::Geode& node )
{
    unsigned int idx;
    for( idx=0; idx < node.getNumDrawables(); idx++ )
    {
        osg::Drawable* draw = node.getDrawable( idx );

        QueryDrawCB* qdcb = new QueryDrawCB;
        draw->setDrawCallback( qdcb );
        draw->setCullCallback( qdcb );

        osgwTools::CountsVisitor cv;
        cv.apply( draw );
        qdcb->setNumVertices( cv.getVertices() );
    }

    traverse( node );
}


// osgwQuery
}
