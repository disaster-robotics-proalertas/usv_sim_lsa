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

#include <osgwQuery/QueryComputation.h>
#include <osgwQuery/QueryAPI.h>
#include <osgwQuery/QueryBenchmarks.h>
#include <osgwTools/Shapes.h>
#include <osgwTools/Transform.h>
#include <OpenThreads/ScopedLock>
#include <osgUtil/CullVisitor>
#include <osg/ComputeBoundsVisitor>
#include <osg/PolygonOffset>
#include <osg/ColorMask>
#include <osg/Depth>

#include <osg/io_utils>


namespace osgwQuery
{


// Statics
osg::ref_ptr< osg::StateSet > QueryComputation::s_queryStateSet( NULL );

QueryComputation::CscrOiMap QueryComputation::s_CscrOiMap;


QueryComputation::QueryComputation( osgwQuery::QueryStats* debugStats )
  : _initialized( false ),
    _numVertices( 0 ),
    _lastQueryFrame( 0 ),
    _lastCullFrame( 0 ),
    _debugStats( debugStats )
{
}
QueryComputation::QueryComputation( const QueryComputation& rhs, const osg::CopyOp& copyop )
  : _initialized( false ),
    _numVertices( rhs._numVertices ),
    _lastQueryFrame( rhs._lastQueryFrame ),
    _lastCullFrame( rhs._lastCullFrame )
{
}


bool QueryComputation::cullOperation( osg::NodeVisitor* nv, osg::RenderInfo& renderInfo )
{
    if( !_initialized )
    {
        // Must hold lock: init() creates a static StateSet shared by all
        // QueryComputation objects. Only need to create that StateSet once.
        OpenThreads::ScopedLock< OpenThreads::Mutex > lock( _lock );

        init( nv );
        _initialized = true;
    }

    const unsigned int contextID = renderInfo.getState()->getContextID();
    QueryDrawableSet& qds = _queryDrawables[ nv->getNodePath() ];
    osg::ref_ptr< osg::Geometry >& queryDrawable = qds[ contextID ];
    if( !queryDrawable.valid() )
        queryDrawable = initQueryDrawable( nv );

    osgUtil::CullVisitor* cv = static_cast< osgUtil::CullVisitor* >( nv );
    osg::RefMatrix* view = cv->getModelViewMatrix();
    const osg::Camera* cam = cv->getCurrentCamera();


    // Obtain cull-time constants (possibly different per cull in multidisplay).
    //     Distance from the Drawble to the viewpoint.
    const double dOi = cv->getDistanceFromEyePoint( _worldBB.center(), false );
    //     Width and height of the viewport.
    const osg::Viewport* vp = cam->getViewport();
    const double width = vp->width();
    const double height = vp->height();
    //     Half of the vertical field of view.
    osg::Matrix proj = cam->getProjectionMatrix();
    double fovy, aspect, zNear, zFar;
    proj.getPerspective( fovy, aspect, zNear, zFar );
    const double thetaOver2 = osg::DegreesToRadians( fovy * .5 );


    // We should only be here if the cull visitor visits our associated Node.
    // That should only happen if our Node isn't frustum culled. Determine
    // if the current frame# is significantly different from the last time
    // we were here, then record the current frame#.
    const unsigned int currentFrame = nv->getFrameStamp()->getFrameNumber();
    bool wasFrustumCulled( _lastCullFrame < currentFrame - 2 );
    _lastCullFrame = currentFrame;
    // Note that we will explicitly set wasFrustumCulled=false
    // if it turns out we have an active query.

    osgwQuery::QueryBenchmarks* qb = osgwQuery::getQueryBenchmarks( contextID, &renderInfo );
    osgwQuery::QueryAPI* qapi = osgwQuery::getQueryAPI( contextID );

    QueryDrawCallback* qdc = static_cast< QueryDrawCallback* >( queryDrawable->getDrawCallback() );

    // Retrieve any pending query result.
    if( qdc->_queryActive )
    {
        GLuint result;
        const GLuint id = qdc->_queryObject->getID( contextID );
        qapi->glGetQueryObjectuiv( id, GL_QUERY_RESULT, &result );
        osg::notify( osg::INFO ) << " ID: " << id << " Result: " << result << ", numV " << _numVertices << std::endl;
        qdc->_wasOccluded = ( result < 25 ); // TBD need configurable threshold.
        qdc->_queryActive = false;

        // If query was active, we were _not_ frustum culled.
        wasFrustumCulled = false;
    }

    // Report if frustum culled
    if( _debugStats.valid() && wasFrustumCulled )
        _debugStats->incFrustum();

    const double cbbOiSqRt = 1. / ( dOi * sqrt( width / height ) * 2. * tan( thetaOver2 ) );
    const double cbbOi = cbbOiSqRt * cbbOiSqRt * _AbbOiOver6;

    // Compute pcovOi, the probability that this Node is covered.
    double pcovOi;
    const double cscrOi = getCscrOi( cam, contextID );
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
    if( !qdc->_wasOccluded )
        // previously visible
        pocclOi = 0.5 * pcovOi * pcovOi;
    else if( wasFrustumCulled )
        pocclOi = pcovOi;
    else /* previously occluded */
        pocclOi = 1.;
    if( _debugStats.valid() )
        _debugStats->setPoccl( (float)pocclOi );



    /** Implements the Guthe paper'a "QueryReasonable" function, pseudocode
    in Guthe Figure 5, described in Guthe section 4. */
    bool queryReasonable( false );


    double cOi = cbbOi / _RcovOi;
    // TBD need max of triangle cost or fragment cost.
    const double pixels = width * height * cOi;
    const double fragRenderTime = pixels * qb->_trFragment;
    // TBD QueryBenchmark should time per-vertex cost.
    // TBD vertRenderTime is a constant.
    const double vertRenderTime = _numVertices * qb->_trTriangle;
    const double renderTime = osg::maximum< double >( fragRenderTime, vertRenderTime ) + qb->_trSetup;
    const double queryTime = pixels * qb->_toFragment;// + qb->_toSetup;

    // TBD hardcoded literanl should be set by application or detected automatically with
    // a node visitor.
    const double sceneDepth( 3. );
    const double nodeCost = queryTime;
    const double nodeBenefit = pocclOi * ( renderTime - queryTime ) / sceneDepth;

    unsigned int framesSinceLastQuery = currentFrame - _lastQueryFrame;



    if( dOi < _worldBB.radius() )
    {
        // Not part of Guthe, but required for correct functionality in
        // real apps, especially OSG with autocompute near/far enabled.
        // Eyepoint is quite close to (or inside) bounding volome.
        // Don't waste time issuing a query, just traverse the children.
        queryReasonable = false;
        qdc->_wasOccluded = false;
    }

    else if( _numVertices < 1 )
    {
        // This is not part of Guthe, but we do occasionally encounter models like this.
        // Don't do a query, just let OSG handle it however OSG handles it.
        queryReasonable = false;
    }

    else if( ( renderTime < queryTime ) /* ||
            ( sum of child costs < node cost ) */ )
    {
        queryReasonable = false;
        osg::notify( osg::INFO ) << "Case 1: False. renderTime < queryTime" << std::endl;

        if( _debugStats.valid() )
            _debugStats->incRtLessQt();
    }

    else if( qdc->_wasOccluded )
    {
        queryReasonable = true;
        osg::notify( osg::INFO ) << "Case 2: True. Was occluded" << std::endl;

        if( _debugStats.valid() )
            _debugStats->incOccluded();
    }

    else if( nodeCost > ( framesSinceLastQuery * nodeBenefit ) )
    {
        queryReasonable = false;
        osg::notify( osg::INFO ) << "Case 3: False. cost > benefit" << std::endl;

        if( _debugStats.valid() )
            _debugStats->incCGreaterB();
    }

    else
    {
        queryReasonable = true;
        osg::notify( osg::INFO ) << "Case 4: True. default" << std::endl;
    }
    // NOTE: Following is the portion of the Guthe algorithm that is not implemented,
    // as OSG doesn't support dynamically editing the render graph.
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
        osg::StateSet* stateSet = queryDrawable->getStateSet();
        if( stateSet )
            cv->pushStateSet( stateSet );

        cv->updateCalculatedNearFar( *view, *queryDrawable, false );

        // Issue query. Add the query drawable to the render graph just as
        // if CullVisitor had encountered it while processing a Geode.
        const float depth = _worldBB.valid() ? cv->getDistanceFromEyePoint( _worldBB.center(), false ) : 0.f;
        if( osg::isNaN( depth ) )
        {
            osg::notify(osg::NOTICE)<<"QueryComputation: detected NaN,"<<std::endl
                                    <<"    depth="<<depth<<", center=("<<_worldBB.center()<<"),"<<std::endl;
        }
        else
        {
            osg::notify( osg::INFO ) << "Issued query" << std::endl;
            cv->addDrawableAndDepth( queryDrawable.get(), view, depth );
        }

        if( stateSet )
            cv->popStateSet();

        _lastQueryFrame = currentFrame;

        if( _debugStats.valid() )
            _debugStats->incQueries();
    }


    // Increment CscrOi for next Drawable.
    // TBD probable threading issue. Multiple cull threads executing
    // this function will result in corrupting cscrOi. Need cscrOi
    // per thread!
    setCscrOi( cscrOi + ( ( 1. - cscrOi ) * cbbOi ), cam, contextID );

    osg::notify( osg::INFO ) << "  Was occluded? " << std::boolalpha << qdc->_wasOccluded << " numV " << _numVertices << std::endl;
    return( !qdc->_wasOccluded );
}

double QueryComputation::getCscrOi( const osg::Camera* cam, unsigned int contextID )
{
    const CameraContext camCtx = CameraContext( cam, contextID );
    CscrOiMap::const_iterator it = s_CscrOiMap.find( camCtx );
    if( it != s_CscrOiMap.end() )
        return( it->second );
    else
        return( s_CscrOiMap[ camCtx ] = 0. );
}
void QueryComputation::setCscrOi( double c, const osg::Camera* cam, unsigned int contextID )
{
    const CameraContext camCtx = CameraContext( cam, contextID );
    s_CscrOiMap[ camCtx ] = c;
}


void QueryComputation::init( osg::NodeVisitor* nv )
{
    _worldBB = osgwTools::transform( osg::computeLocalToWorld( nv->getNodePath() ), _bb );

    //
    // Initialize a static StateSet used by all QueryComputation's queryDrawable objects.
    if( !( s_queryStateSet.valid() ) )
    {
        s_queryStateSet = new osg::StateSet;

        osg::PolygonOffset* po = new osg::PolygonOffset( -1.f, -1.f );
        s_queryStateSet->setAttributeAndModes( po );

        osg::ColorMask* cm = new osg::ColorMask( false, false, false, false );
        s_queryStateSet->setAttributeAndModes( cm );

        // Sadly, there is no way in OSG to set the depth mask without also setting
        // the depth function and the viewport depth range values. So we go ahead and
        // set them (we have no choice) and hope for the best.
        osg::Depth* depth = new osg::Depth( osg::Depth::LESS, 0., 1., false );
        s_queryStateSet->setAttributeAndModes( depth );
    }


    //
    // Compute and store constants defined in Guthe, section 3.1.

    // 1/6th of the bounding box surface area.
    osg::Vec3 extents = _bb._max - _bb._min;
    const double abbOi = ( 2. * extents[ 0 ] * extents[ 1 ] ) +
        ( 2. * extents[ 1 ] * extents[ 2 ] ) +
        ( 2. * extents[ 2 ] * extents[ 0 ] );
    _AbbOiOver6 = abbOi / 6.;

    // Ratio of actual object screen area to boulding box screen area.
    const double aOi = 4. * osg::PI * _bb.radius() * _bb.radius();
    _RcovOi = ( 3. / 2. ) * ( aOi / abbOi );
}

osg::Geometry* QueryComputation::initQueryDrawable( osg::NodeVisitor* nv )
{
    //
    // Create box geometry for use as query geometry.

    osg::Vec3 extents = _bb._max - _bb._min;
    osg::Vec3 halfExtents = extents / 2.;
    osg::Geometry* geom = osgwTools::makePlainBox( halfExtents );

    QueryDrawCallback* qdc = new QueryDrawCallback();
    qdc->setName( nv->getNodePath().back()->getName() );
    qdc->attach( this );
    geom->setDrawCallback( qdc );

    // TBD osgwTools Shapes really does need to support non-origin centers.
    // Offset the query geometry box by the bounding box center.
    osg::Vec3Array* v = static_cast< osg::Vec3Array* >( geom->getVertexArray() );
    const osg::Vec3 center = _bb.center();
    unsigned int idx;
    for( idx=0; idx<v->size(); idx++ )
        (*v)[ idx ] += center;

    // Optimize.
    // TBD Possibly display lists are better? Need to try on higher-end hardware.
    geom->setUseDisplayList( false );
    geom->setUseVertexBufferObjects( true );

    geom->setStateSet( s_queryStateSet.get() );

    return( geom );
}



QueryDrawCallback::QueryDrawCallback()
  : osg::Drawable::DrawCallback(),
    _queryActive( false ),
    _wasOccluded( false ),
    _queryObject( new QueryObject() ),
    _nd( NULL )
{
}

QueryDrawCallback::QueryDrawCallback( const QueryDrawCallback& rhs, const osg::CopyOp& copyop )
  : osg::Drawable::DrawCallback( rhs ),
    _queryActive( rhs._queryActive ),
    _wasOccluded( rhs._wasOccluded ),
    _queryObject( rhs._queryObject ),
    _nd( rhs._nd )
{
}

void QueryDrawCallback::drawImplementation( osg::RenderInfo& renderInfo, const osg::Drawable* drawable ) const
{
    if( _nd == NULL )
        return;

    const unsigned int contextID = renderInfo.getState()->getContextID();
    osgwQuery::QueryAPI* qapi = osgwQuery::getQueryAPI( contextID );

    const GLuint id = _queryObject->getID( contextID );
    osg::notify( osg::INFO ) << " ID: " << id << std::endl;
    qapi->glBeginQuery( GL_SAMPLES_PASSED, id );

    drawable->drawImplementation( renderInfo );

    qapi->glEndQuery( GL_SAMPLES_PASSED );
    _queryActive = true;
}

void QueryDrawCallback::attach( osgwQuery::QueryComputation* nd )
{
    _nd = nd;
}


// osgwQuery
}
