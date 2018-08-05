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

#include <osgwQuery/QueryUtils.h>
#include <osgwQuery/QueryAPI.h>
#include <osgwQuery/QueryBenchmarks.h>
#include <osgwTools/Shapes.h>
#include <osgwTools/CountsVisitor.h>
#include <osgwTools/Transform.h>
#include <OpenThreads/ScopedLock>
#include <osgUtil/CullVisitor>
#include <osgUtil/RenderBin>

#include <osg/io_utils>
#include <osgwTools/Version.h>

#include <string>


namespace osgwQuery
{


#if( OSGWORKS_OSG_VERSION < 30000 )
    // Front-to-back bin is already defined in v3.0.0 and later.
    osgUtil::RegisterRenderBinProxy s_registerQueryBinProxy( _QUERY_FRONT_TO_BACK_BIN_NAME,
        new osgUtil::RenderBin( osgUtil::RenderBin::SORT_FRONT_TO_BACK ) );
#endif


QueryCullCallback::QueryCullCallback()
  : osg::NodeCallback(),
    _node( NULL ),
    _nd( NULL )
{
}
QueryCullCallback::QueryCullCallback( const QueryCullCallback& rhs, const osg::CopyOp& copyop )
  : osg::NodeCallback( rhs ),
    _node( rhs._node ),
    _nd( rhs._nd )
{
}

void QueryCullCallback::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
    if( ( _node == NULL ) || ( _nd == NULL ) )
        return;

    osgUtil::CullVisitor* cv = static_cast< osgUtil::CullVisitor* >( nv );
    osg::RenderInfo& renderInfo = cv->getRenderInfo();

    bool traverseChildren = _nd->cullOperation( nv, renderInfo );
    if( !traverseChildren )
        return;

    traverse( node, nv );
}

void QueryCullCallback::attach( osg::Node* node, osgwQuery::QueryComputation* nd )
{
    _node = node;
    _nd = nd;
}



CameraResetCallback::CameraResetCallback()
{
}
void CameraResetCallback::operator()( osg::RenderInfo& renderInfo ) const
{
    osg::Camera* cam = renderInfo.getCurrentCamera();
    unsigned int contextID = renderInfo.getState()->getContextID();
    QueryComputation::setCscrOi( 0., cam, contextID );
}



void AddQueries::apply( osg::Group& node )
{
    if( node.getName() == std::string( "__QueryStats" ) )
        // This is the QueryStats subtree. Don't instrument it with any OQ stuff.
        return;

    if( node.getCullCallback() != NULL )
    {
        traverse( node );
        return;
    }

    // Do not add callbacks to redundant Groups because the parent Group's
    // bounding volume (and query geometry) would also be redundant.
    // This Group is not redundant if it has no parents, or if its parents
    // are all Cameras, or if any one of its non-Camera parents has
    // more than one child. Otherwise, it's redundant.
    bool redundantGroup( false );
    unsigned int parentsWithOneChild( 0 );
    const unsigned int numParents( node.getNumParents() );
    unsigned int idx;
    for( idx=0; idx < numParents; idx++ )
    {
        osg::Group* parent = node.getParent( idx );
        bool parentIsCamera = ( dynamic_cast< osg::Camera* >( parent ) != NULL );
        if( parentIsCamera )
            continue;
        if( parent->getNumChildren() == 1 )
        {
            parentsWithOneChild++;
            // If all parents have one child, then we are redundant.
            if( numParents == parentsWithOneChild )
                redundantGroup = true;
        }
    }
    if( redundantGroup )
    {
        if( ( _qs != NULL ) && ( &node == _qs->getNode() ) )
            osg::notify( osg::ALWAYS ) << "Debug: Unable to add QueryStats to redundant Group \"" << node.getName() << "\"." << std::endl;
        traverse( node );
        return;
    }

    // Create QueryComputation for this node.
    // Add a QueryStats only if a) we have one and
    // b) the node addresses match.
    osgwQuery::QueryStats* debugStats( NULL );
    if( ( _qs != NULL ) && ( &node == _qs->getNode() ) )
    {
        osg::notify( osg::ALWAYS ) << "Debug: Adding QueryStats to node \"" << node.getName() << "\"." << std::endl;
        debugStats = _qs;
    }
    QueryComputation* nd = new QueryComputation( debugStats );

    QueryCullCallback* qcc = new QueryCullCallback();
    qcc->setName( node.getName() );
    qcc->attach( &node, nd );
    node.setCullCallback( qcc );

    _queryCount++;

    traverse( node );
}
void AddQueries::apply( osg::Geode& node )
{
    traverse( node );

    osgwTools::CountsVisitor cv;
    node.accept( cv );
    const unsigned int numVertices = cv.getVertices();
    const osg::BoundingBox& bb = node.getBoundingBox();
    addDataToNodePath( getNodePath(), numVertices, bb );
}
void AddQueries::apply( osg::Camera& node )
{
    if( node.getCullCallback() != NULL )
    {
        traverse( node );
        return;
    }

    CameraResetCallback* crc = new CameraResetCallback();
    // TBD use the osgWorks composite post-draw callback.
    node.setPostDrawCallback( crc );

    traverse( node );
}

void AddQueries::addDataToNodePath( osg::NodePath& np, unsigned int numVertices, const osg::BoundingBox& bb )
{
    // For every Node in the NodePath that has a QueryCullCallback and a 
    // QueryComputation object, add the number of vertices and the transformed
    // bounding box to that Node's QueryComputation.

    osg::NodePath localNP;
    osg::NodePath::reverse_iterator rit;
    for( rit=np.rbegin(); rit!=np.rend(); rit++ )
    {
        osg::NodeCallback* cb = (*rit)->getCullCallback();
        QueryCullCallback* qcc = dynamic_cast< QueryCullCallback* >( cb );

        osgwQuery::QueryComputation* nd( NULL );
        if( qcc != NULL )
            nd = qcc->getQueryComputation();
        if( nd != NULL )
        {
            nd->setNumVertices( nd->getNumVertices() + numVertices );

            osg::BoundingBox xformBB = osgwTools::transform( osg::computeLocalToWorld( localNP ), bb );
            osg::BoundingBox ndBB = nd->getBoundingBox();
            ndBB.expandBy( xformBB );
            nd->setBoundingBox( ndBB );
        }

        // Yuck. std::vector doesn't support push_back().
        //localNP.push_front( *rit );
        localNP.resize( localNP.size() + 1 );
        unsigned int idx;
        for( idx=localNP.size()-1; idx>0; idx-- )
            localNP[ idx ] = localNP[ idx-1 ];
        localNP[ 0 ] = *rit;
    }
}



void RemoveQueries::apply( osg::Node& node )
{
    osg::NodeCallback* cb = node.getCullCallback();
    QueryCullCallback* qcc = dynamic_cast< QueryCullCallback* >( cb );

    if( qcc != NULL )
        node.setCullCallback( NULL );

    traverse( node );
}

void RemoveQueries::apply( osg::Camera& node )
{
    osg::Camera::DrawCallback* cb = node.getPostDrawCallback();
    CameraResetCallback* crc = dynamic_cast< CameraResetCallback* >( cb );

    if( crc != NULL )
        node.setPostDrawCallback( NULL );

    traverse( node );
}


// osgwQuery
}
