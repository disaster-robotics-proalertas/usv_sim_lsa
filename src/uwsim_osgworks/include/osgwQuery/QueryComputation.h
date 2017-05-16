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

#ifndef __OSGWQUERY_QUERY_COMPUTATION_H__
#define __OSGWQUERY_QUERY_COMPUTATION_H__ 1


#include <osgwQuery/Export.h>
#include <osgwQuery/QueryObject.h>
#include <osgwQuery/QueryStats.h>
#include <osg/Drawable>
#include <OpenThreads/Mutex>

#include <osg/Node>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/NodeVisitor>
#include <osg/buffered_value>

#include <map>


namespace osgwQuery
{


/** \defgroup GutheQuery OpenGL occlusion query support.
\brief An OSG implementation of the Guthe occlusion query algorithm.

The Guthe algorithm was published in "Near Optimal Hierarchical Culling:
Performance Driven Use of Hardware Occlusion Queries", Eurographics, 2006.

The QueryComputation class implements the Guthe algorithm in the
QueryComputation::cullOperation() method. The QueryCullCallback class calls
cullOperation() during the cull traversal. If cullOperation() determines that
an occlusion query should be performed, it adds a query Geometry to the render
graph. At draw time, the Geometry's QueryDrawCallback issues the necessary
OpenGL occlusion query commands.

This is only a partial implementation of the Guthe algorithm due to
general-purpose rendering limitations within OSG itself. The parts of the Guthe
algorithm aren't implemented:
\li Guthe combines culling and drawing into a single operation, and requires
issuing draw commands for foreground objects before making cull decisions about more
distant objects. Cull and draw are two distinct operations in OSG and can't be
combined. Combining them would require OSG to dynamically alter the contents of
the render graph at draw-time, adding (or not adding) scene graph children based on 
the occlusion state of a given parent. OSG's render graph contains no links back
to the source scene graph, so this type of rendering is impossible in OSG.
\li Guthe requires rendering in front-to-back order, which this implementation
supports. However, Guthe treats cull and draw as synonymous operations (see above),
and therefore also requires that cull occur in front-to-back order. Indeed, this is
required for correct computation of P<sub>cov</sub>O<sub>i</sub>, the probability that a given object
is covered. However, OSG's CullVisitor traverses children in child order and
oesn't support traversing children in front-to-back order. As a result, this implementation
calculates P<sub>cov</sub>O<sub>i</sub> incorrectly in most cases, causing non-optimal
decisions about when to issue queries.

The algorithm requires hardware performance characterization. The QueryBenchmarks
class supports gathering these characteristics. The InitCallback class allows the
characteristics to be gathered in a Camera pre-draw callback.

The AddQueries NodeVisitor adds QueryCullCallback objects to nodes in the scene graph.
Nodes receive a QueryCullCallback if they are Groups or derived from Group, not
edundant Groups, and not Camera nodes. The AddQueries visitor adds a
CameraResetCallback to Camera nodes to reset the Guthe coverage parameter
before proceeding the the Camera subtree's cull.

The queryStats and QueryStatsHandler classes are available as debugging
aids.

For an example of using the Guthe algorithm, see the testqueryguthe test program.
The data directory also contains four model test files.
\code
testqueryguthe octest0.osg
testqueryguthe octest1.osg
testqueryguthe octest2.osg
testqueryguthe octest3.osg
\endcode

To see the QueryStats class in action, try this:
\code
testqueryguthe octest3.osg --debug negXTruck
\endcode

*/
/*@{*/

    
/** \class QueryComputation QueryComputation.h <osgwQuery/QueryComputation.h>
\brief A support struct for the Guthe occlusion query algorithm.

This class attempts to implement the algorithm described in "Near Optimal
Hierarchical Culling: Performance Driven Use of Hardware Occlusion Queries"
by Guthe, Balázs, and Klein, Eurographics 2006.
*/
class OSGWQUERY_EXPORT QueryComputation : public osg::Object
{
public:
    QueryComputation( osgwQuery::QueryStats* debugStats=NULL );
    QueryComputation( const QueryComputation& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgwQuery,QueryComputation);

    /** \brief Implements Guthe algorithm and tells calling code whether to render children.

    NOTE: Guthe assumes a concurrent cull/draw with an active rendering context. The
    cullOperation() method requires an active context in order to retrieve the active query
    result. If you call this function without an active context, results are undefined.
    Many osgViewer threading models do *not* have an active rendering context during cull
    and will not work with this algorithm.

    If the algorithm determines that a query is reasonable, it will insert a Drawable into
    the render graph and return false. The Drawable represents the bounding box of the
    node's subgraph and uses a draw callback to issue the OpenGL query commands.

    \return true if the calling node should continue traversing its children and adding
    them to the render graph. False if the calling code should not add its children to
    the render graph. */
    virtual bool cullOperation( osg::NodeVisitor* nv, osg::RenderInfo& renderInfo );

    /** Access the static C<sub>scr</sub>O<sub>i</sub> parameter, which is kept in
    a static map keyed by the context/Camera pair. */
    static double getCscrOi( const osg::Camera* cam, unsigned int contextID );
    static void setCscrOi( double c, const osg::Camera* cam, unsigned int contextID );

    /** Access the number of vertices contained in the subgraph rooted at the
    node associated with this QueryComputation instance. */
    void setNumVertices( unsigned int numVertices ) { _numVertices = numVertices; }
    unsigned int getNumVertices() const { return( _numVertices ); }

    /** Access the BoundingBox contained in the subgraph rooted at the
    node associated with this QueryComputation instance. If the node is
    a Transform, the BoundingBox is in untransformed (subgraph-local)
    coordinates. */
    void setBoundingBox( const osg::BoundingBox& bb ) { _bb = bb; }
    const osg::BoundingBox& getBoundingBox() const { return( _bb ); }

protected:
    void init( osg::NodeVisitor* nv );
    bool _initialized;

    osg::Geometry* initQueryDrawable( osg::NodeVisitor* nv );

    OpenThreads::Mutex _lock;
    static osg::ref_ptr< osg::StateSet > s_queryStateSet;

    unsigned int _numVertices;
    osg::BoundingBox _bb;
    osg::BoundingBox _worldBB;

    // Guthe algorithm constants and variables.
    // Note: Must hold _lock when writing these values during cull.
    //   Surface area of the bounding box divided by 6.
    double _AbbOiOver6;
    //   Ratio: actual object coverage / bounding box coverage.
    double _RcovOi;
    //   Last frame for which a query was issued.
    unsigned int _lastQueryFrame;
    // Used to determine if the node was frustum culled. This is a Guthe
    // constant (indirectly).
    unsigned int _lastCullFrame;

    // Guthe variable to track accumulated coverage during front-to-back
    // rendering. Must be set to 0. at the start of each frame.
    typedef std::pair< const osg::Camera*, unsigned int /*ctx*/ > CameraContext;
    typedef std::map< CameraContext, double > CscrOiMap;
    static CscrOiMap s_CscrOiMap;


    typedef osg::buffered_object< osg::ref_ptr< osg::Geometry > > QueryDrawableSet;
    typedef std::map< osg::NodePath, QueryDrawableSet > QueryDrawables;
    QueryDrawables _queryDrawables;

    osg::ref_ptr< osgwQuery::QueryStats > _debugStats;
};


/** \class QueryDrawCallback QueryComputation.h <osgwQuery/QueryComputation.h>
\brief A draw-time callback that wraps begin/end query calls around the drawImplementation.
*/
class OSGWQUERY_EXPORT QueryDrawCallback : public osg::Drawable::DrawCallback
{
public:
    QueryDrawCallback();
    QueryDrawCallback( const QueryDrawCallback& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgwQuery,QueryDrawCallback);

    virtual void drawImplementation( osg::RenderInfo& renderInfo, const osg::Drawable* drawable ) const;

    void attach( osgwQuery::QueryComputation* nd );

    mutable bool _queryActive;
    bool _wasOccluded;
    osg::ref_ptr< osgwQuery::QueryObject > _queryObject;

protected:
    osgwQuery::QueryComputation* _nd;
};

/*@}*/


// osgwQuery
}

// __OSGWQUERY_QUERY_COMPUTATION_H__
#endif
