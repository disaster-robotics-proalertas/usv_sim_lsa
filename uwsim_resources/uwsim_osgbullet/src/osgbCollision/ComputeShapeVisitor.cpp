/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgBullet is (C) Copyright 2009-2012 by Kenneth Mark Bryden
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

#include <osgbCollision/ComputeShapeVisitor.h>
#include <osg/ComputeBoundsVisitor>
#include <osg/Geode>
#include <osg/Notify>
#include <osgwTools/Transform.h>
#include <osgwTools/Version.h>
#if( OSGWORKS_OSG_VERSION < 30109 )
#  include <osgwTools/ShortEdgeOp.h>
#endif()
#include <osgwTools/ReducerOp.h>
#include <osgwTools/GeometryModifier.h>
#include <osgbCollision/Utils.h>
#include <osgwTools/AbsoluteModelTransform.h>


namespace osgbCollision
{


ComputeShapeVisitor::ComputeShapeVisitor( const BroadphaseNativeTypes shapeType,
        const osgbCollision::AXIS axis, const unsigned int reductionLevel,
        osg::NodeVisitor::TraversalMode traversalMode )
  : osg::NodeVisitor( traversalMode ),
    _shapeType( shapeType ),
    _axis( axis ),
    _reductionLevel( reductionLevel ),
    _shape( new btCompoundShape() )
{
}

void ComputeShapeVisitor::apply( osg::Group& node )
{
    // If this is the root node, the bounding sphere will be invalid. Compute it.
    if( !( _bs.valid() ) )
        _bs = node.getBound();

    traverse( node );
}
void ComputeShapeVisitor::apply( osg::Node& node )
{
    // If this is the root node, the bounding sphere will be invalid. Compute it.
    if( !( _bs.valid() ) )
        _bs = node.getBound();

    traverse( node );
}
void ComputeShapeVisitor::apply( osg::Transform& node )
{
    // If this is the root node, the bounding sphere will be invalid. Compute it.
    if( !( _bs.valid() ) )
        _bs = node.getBound();

    /* Override apply(Transform&) to avoid processing AMT nodes. */
    const bool nonAMT = ( dynamic_cast< osgwTools::AbsoluteModelTransform* >( &node ) == NULL );
    if( nonAMT )
        _localNodePath.push_back( &node );

    traverse( node );

    if( nonAMT )
        _localNodePath.pop_back();
}

void ComputeShapeVisitor::apply( osg::Geode& node )
{
    // If this is the root node, the bounding sphere will be invalid. Compute it.
    if( !( _bs.valid() ) )
        _bs = node.getBound();

    osg::Matrix m = osg::computeLocalToWorld( _localNodePath );
    createAndAddShape( node, m );
}

btCollisionShape* ComputeShapeVisitor::getShape()
{
    return( _shape );
}
const btCollisionShape* ComputeShapeVisitor::getShape() const
{
    return( _shape );
}

void ComputeShapeVisitor::createAndAddShape( osg::Node& node, const osg::Matrix& m )
{
    osg::notify( osg::DEBUG_INFO ) << "In createAndAddShape" << std::endl;

    btCollisionShape* child = createShape( node, m );
    if( child )
    {
        btCompoundShape* master = static_cast< btCompoundShape* >( _shape );
        btTransform transform; transform.setIdentity();
        master->addChildShape( transform, child );
    }
}
btCollisionShape* ComputeShapeVisitor::createShape( osg::Node& node, const osg::Matrix& m )
{
    osg::notify( osg::DEBUG_INFO ) << "In createShape" << std::endl;

    // Make a copy of the incoming node and its data. The copy witll be transformed by the
    // specified matrix, and possibly geometry reduced.
    if( node.asGeode() == NULL )
    {
        osg::notify( osg::WARN ) << "ComputeShapeVisitor encountered non-Geode." << std::endl;
        return( NULL );
    }
    osg::Geode* geodeCopy = new osg::Geode( *( node.asGeode() ), osg::CopyOp::DEEP_COPY_ALL );
    osgwTools::transform( m, geodeCopy->asGeode() );

    btCollisionShape* collision( NULL );
    osg::Vec3 center;

    switch( _shapeType )
    {
    case BOX_SHAPE_PROXYTYPE:
    {
        osg::ComputeBoundsVisitor cbv;
        geodeCopy->accept( cbv );
        osg::BoundingBox bb = cbv.getBoundingBox();
        center = bb.center();
        collision = osgbCollision::btBoxCollisionShapeFromOSG( geodeCopy, &bb );
        break;
    }
    case SPHERE_SHAPE_PROXYTYPE:
    {
        osg::BoundingSphere bs = geodeCopy->getBound();
        center = bs.center();
        collision = osgbCollision::btSphereCollisionShapeFromOSG( geodeCopy );
        break;
    }
    case CYLINDER_SHAPE_PROXYTYPE:
    {
        osg::BoundingSphere bs = geodeCopy->getBound();
        center = bs.center();
        collision = osgbCollision::btCylinderCollisionShapeFromOSG( geodeCopy, _axis );
        break;
    }
    case TRIANGLE_MESH_SHAPE_PROXYTYPE:
    {
        // Do _not_ compute center of bounding sphere for tri meshes.

        // Reduce geometry.
        reduce( *geodeCopy );
        collision = osgbCollision::btTriMeshCollisionShapeFromOSG( geodeCopy );
        break;
    }
    case CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE:
    {
        // Do _not_ compute center of bounding sphere for tri meshes.
        
        // Reduce geometry.
        reduce( *geodeCopy );
        collision = osgbCollision::btConvexTriMeshCollisionShapeFromOSG( geodeCopy );
        break;
    }
    case CONVEX_HULL_SHAPE_PROXYTYPE:
    {
        // Do _not_ compute center of bounding sphere for tri meshes.
        collision = osgbCollision::btConvexHullCollisionShapeFromOSG( geodeCopy );
        break;
    }
    default:
    {
        osg::notify( osg::FATAL ) << "ComputeShapeVisitor: Error, unknown shape type, using tri mesh." << std::endl;
        break;
    }
    }

    if( collision && ( center != osg::Vec3( 0., 0., 0. ) ) )
    {
        btTransform trans; trans.setIdentity();
        trans.setOrigin( osgbCollision::asBtVector3( center ) );
        btCompoundShape* masterShape = new btCompoundShape();
        masterShape->addChildShape( trans, collision );
        collision = masterShape;
    }

    return( collision );
}

void ComputeShapeVisitor::reduce( osg::Node& node )
{
    if( !( _bs.valid() ) )
    {
        osg::notify( osg::WARN ) << "ComputeShapeVisitor: Can't reduce with invalid bound." << std::endl;
        return;
    }

    float seFeature;
    float sePercent;
    float grpThreshold;
    float edgeError;
    switch( _reductionLevel )
    {
    case 1:
        seFeature = .15f;
        sePercent = .9f;
        grpThreshold = 8.f;
        edgeError = 8.f;
        break;
    case 2:
        seFeature = .25f;
        sePercent = .6f;
        grpThreshold = 17.f;
        edgeError = 17.f;
        break;
    case 3:
        seFeature = .35f;
        sePercent = .3f;
        grpThreshold = 28.f;
        edgeError = 28.f;
        break;
    case 0:
    default:
        // No reduction.
        return;
        break;
    }
    seFeature *= _bs.radius() * 2.;

    osg::notify( osg::DEBUG_FP ) << "ComputeShapeVisitor: Reducing..." << std::endl;
#if( OSGWORKS_OSG_VERSION < 30109 )
    {
        osgwTools::ShortEdgeOp* seOp = new osgwTools::ShortEdgeOp( sePercent, seFeature );
        seOp->setDoTriStrip( false );
        seOp->setMinPrimitives( 1 );

        osgwTools::GeometryModifier modifier( seOp );
        node.accept( modifier );
        modifier.displayStatistics( osg::notify( osg::DEBUG_FP ) );
    }
#endif

    {
        osgwTools::ReducerOp* redOp = new osgwTools::ReducerOp;
        redOp->setGroupThreshold( grpThreshold );
        redOp->setMaxEdgeError( edgeError );

        osgwTools::GeometryModifier modifier( redOp );
        node.accept( modifier );
        modifier.displayStatistics( osg::notify( osg::DEBUG_FP ) );
    }
}



// osgbCollision
}
