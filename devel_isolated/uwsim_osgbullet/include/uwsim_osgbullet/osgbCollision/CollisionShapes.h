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

#ifndef __OSGBCOLLISION_COLLISION_SHAPES_H__
#define __OSGBCOLLISION_COLLISION_SHAPES_H__ 1


#include <osgbCollision/Export.h>
#include <osg/BoundingBox>

#include <btBulletCollisionCommon.h>

// Forward
namespace osg {
    class Node;
    class Geometry;
}


namespace osgbCollision
{


/** \defgroup collisionshapes Collision Shape Conversion
\brief Routines for converting between OSG geometric data and Bullet collision shapes.

*/
/**@{*/

/** \brief Enumerant to denote the major axis of a cylinder.

These are the values that btCylinderShape::getUpAxis() returns
for the three different cylinder major axes.
*/
enum AXIS
{
    X=0, Y=1, Z=2
};


/** \brief Return a Bullet box collision shape that approximates the specified OSG geometry.

The box is untransformed. Bullet's box collision shape uses the box extents only.
(btBoxShape is always centered on the origin. See btCompoundShapeFromBounds() to create
the box as a transformed child shape within a btCompoundShape.)

If the calling code has already computed the bounding box extents, pass this information
as the \c bb parameter, and this function will use that information to create the collision shape.
If you do not pass a \c bb parameter, this function uses the \c osg::ComputeBoundsVisitor to
determine the bounding box extents. */
OSGBCOLLISION_EXPORT btBoxShape* btBoxCollisionShapeFromOSG( osg::Node* node, const osg::BoundingBox* bb=NULL );

/** \brief Return a Bullet sphere collision shape that approximates the specified OSG geometry.

The sphere is untransformed. Bullet's sphere collision shape uses the radius only.
(btSphereShape is always centered on the origin. See btCompoundShapeFromBounds() to create
the sphere as a transformed child shape within a btCompoundShape.) */
OSGBCOLLISION_EXPORT btSphereShape* btSphereCollisionShapeFromOSG( osg::Node* node );

/** \brief Return a Bullet cylinder collision shape that approximates the specified OSG geometry.

The cylinder is untransformed. Bullet's cylinder collision shape uses the specified axis and computed radius only.
(btCylinderShape is always centered on the origin. See btCompoundShapeFromBounds() to create
the cylinder as a transformed child shape within a btCompoundShape.) */
OSGBCOLLISION_EXPORT btCylinderShape* btCylinderCollisionShapeFromOSG( osg::Node* node, AXIS axis=Y );

/** \brief Return a Bullet triangle mesh collision shape that approximates the specified OSG geometry.

This function collects all triangles and transforms them by any Transforms in the subgraph rootes at \c node.
*/
OSGBCOLLISION_EXPORT btTriangleMeshShape* btTriMeshCollisionShapeFromOSG( osg::Node* node );

/** \brief Return a Bullet convex triangle mesh collision shape that approximates the specified OSG geometry.

This function collects all triangles and transforms them by any Transforms in the subgraph rootes at \c node.
*/
OSGBCOLLISION_EXPORT btConvexTriangleMeshShape* btConvexTriMeshCollisionShapeFromOSG( osg::Node* node );

/** \brief Return a Bullet convex hull collision shape that approximates the specified OSG geometry.

This function collects all vertices and transforms them by any Transforms in the subgraph rootes at \c node.
*/
OSGBCOLLISION_EXPORT btConvexHullShape* btConvexHullCollisionShapeFromOSG( osg::Node* node );

/** \brief Creates a collision shape for each Geode in the scene graph,
and assembles them into a single btCompoundShape.

If \c shapeType is CYLINDER_SHAPE_PROXYTYPE, \c axis indicates the cylinder axis.

\param reductionLevel If \c shapeType is TRIANGLE_MESH_SHAPE_PROXYTYPE or
CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE, a copy of the input geometry is reduced based on
this value, which is in the range 0 (to reduction) to 3 (aggressive reduction). See
osgbDynamics::CreationRecord::ReductionLevel. Default is 0 (no reduction). */
OSGBCOLLISION_EXPORT btCompoundShape* btCompoundShapeFromOSGGeodes( osg::Node* node,
    const BroadphaseNativeTypes shapeType, const osgbCollision::AXIS axis=Y,
    const unsigned int reductionLevel = 0 );

/** \brief Currently not implemented. Creates a collision shape for each Geode in the scene graph,
and assembles them into a single btCompoundShape. */
OSGBCOLLISION_EXPORT btCompoundShape* btCompoundShapeFromOSGGeometry( osg::Node* node );

/** \brief Creates a transformed box, sphere, or cylinder collision shape.

Bullet supports transformed box, sphere, and cylinder shapes with the btCompoundShape.
This function creates a box, sphere, or cylinder collision shape, and adds it as a child
shape to a btCompoundShape. The child shape transform is the bounding volume translation.

Currently, this function supports only translation. In the future, rotation and
scale could also be supported. */
OSGBCOLLISION_EXPORT btCompoundShape* btCompoundShapeFromBounds( osg::Node* node,
    const BroadphaseNativeTypes shapeType, const osgbCollision::AXIS axis=Y );



/*@{*/
/** \brief Return an OSG representation of the given bullet collision shape.

This is useful for Bullet applications being ported to visualize the physics simulation with OSG. */
OSGBCOLLISION_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btCollisionShape* btShape, const btTransform& trans = btTransform::getIdentity() );
/** \overload */
OSGBCOLLISION_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btBoxShape* btShape, const btTransform& trans = btTransform::getIdentity() );
/** \overload */
OSGBCOLLISION_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btSphereShape* btShape, const btTransform& trans = btTransform::getIdentity() );
/** \overload */
OSGBCOLLISION_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btCylinderShape* btShape, const btTransform& trans = btTransform::getIdentity() );
/** \overload */
OSGBCOLLISION_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btTriangleMeshShape* btShape, const btTransform& trans = btTransform::getIdentity() );
/** \overload */
OSGBCOLLISION_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btConvexTriangleMeshShape* btShape, const btTransform& trans = btTransform::getIdentity() );
/** \overload */
OSGBCOLLISION_EXPORT osg::Node* osgNodeFromBtCollisionShape( const btConvexHullShape* btShape, const btTransform& trans = btTransform::getIdentity() );
/*@}*/


/*@{*/
/** \brief Returns an OSG Geometry to render the specified collision shape.

Uses the osgWorks Shapes utilities to create the box Geometry. */
OSGBCOLLISION_EXPORT osg::Geometry* osgGeometryFromBtCollisionShape( const btBoxShape* btShape );
/** \brief Returns an OSG Geometry to render the specified collision shape.

Uses the osgWorks Shapes utilities to create the sphere Geometry. */
OSGBCOLLISION_EXPORT osg::Geometry* osgGeometryFromBtCollisionShape( const btSphereShape* btSphere );
/**  \brief Returns an OSG Geometry to render the specified collision shape.

Uses the OSG ShapeDrawable utilities to create the cylinder Geometry.
When osgWorks Shapes supports cylinders, the implementation will be modified to
remove the dependency on ShapeDrawable. */
OSGBCOLLISION_EXPORT osg::Geometry* osgGeometryFromBtCollisionShape( const btCylinderShape* btCylinder );
/*@}*/

/**@}*/


// osgbCollision
}


// __OSGBCOLLISION_COLLISION_SHAPES_H__
#endif
