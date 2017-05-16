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

#ifndef __OSGCOLLISION_COMPUTE_SHAPE_VISITOR_H__
#define __OSGCOLLISION_COMPUTE_SHAPE_VISITOR_H__ 1

#include <osgbCollision/Export.h>
#include <osgbCollision/CollisionShapes.h>
#include <btBulletCollisionCommon.h>
#include <osg/NodeVisitor>
#include <osgwTools/Version.h>

#include <string>


namespace osgbCollision
{


/** \class ComputeShapeVisitor ComputeShapeVisitor.h <osgbCollision/ComputeShapeVisitor.h>
\brief A NodeVisitor that creates Bullet collision shapes for each Geode and assembles them
into a single btCompoundShape.

This visitor is designed to work in conjuction with the
\link rigidbody rigid body creation utilities \endlink and is invoked indirectly from
osgbDynamics::createRigidBody() via btCompoundShapeFromOSGGeodes().

To correctly support center of mass and scaling, the source scene graph should
be temporarily multiparented to a MatrixTransform containing the concatenation of
the negative center of mass translation and the scale matrix. See the implementation
of osgbDynamics::createRigidBody(), which does this correctly.

Bullet's box, sphere, and cylinder shapes are always at the origin, so this visitor
creates them as child shapes of a btCompoundShape (which supports a btTransform per child).
Triangle mesh, convex tri mesh, and convex hull are generated from transformed geometric
data and don't require this additional level of indirection.

<b>Work To Be Done</b>

The created collision shape is a btCompoundShape. However, the compound shape
is entirely flat (collision shapes created from each Geode are added directly as
children to the btCompoundShape). Bullet might benefit from a more hierarchical
arrangement. For example, when the visitor encounters a Group during traversal,
it could add a btCompoundShape child, which becomes the new parent for any Geodes
under that Group. We should investigate this before proceeding with this work.

A collision shape per Geode is not necessarily the most accurate way to represent
all OSG scene graphs. Perhaps a collision shape per Geometry, or per PrimitiveSet,
or some combination of all three, would be best. For now, only per-Geode is supported.
*/
class OSGBCOLLISION_EXPORT ComputeShapeVisitor : public osg::NodeVisitor
{
public:
    /** \brief Specifies the shape to create per Geode, axis (if \c shapeType is a cylinder),
    and traversal mode. */
    ComputeShapeVisitor( const BroadphaseNativeTypes shapeType, const osgbCollision::AXIS axis=Y,
        const unsigned int reductionLevel=0,
        osg::NodeVisitor::TraversalMode traversalMode=osg::NodeVisitor::TRAVERSE_ALL_CHILDREN );

#if( OSGWORKS_OSG_VERSION >= 20800 )
    META_NodeVisitor(osgbCollision,ComputeShapeVisitor);
#endif

    /** \brief Computes overall bound of input scene graph for use in geometry reduction. */
    void apply( osg::Group& node );
    /** \brief Computes overall bound of input scene graph for use in geometry reduction. */
    void apply( osg::Node& node );
    /** \brief Builds ComputeShapeVisitor::_localNodePath (a NodePath) from all Transforms, excluding AbsoluteModelTransform.

    This visitor must transform all geometry by Transform nodes in the scene graph before using
    that geometry to create the collision shape. However, in order to be compatible with the
    \link rigidbody rigid body creation utilities, \endlink the visitor can't consider
    AbsoluteModelTransforms in such a transformation, as they ignore all parent transforms.

    To support this, we override NodeVisitor::apply(osg::Transform&) to build our own
    NodePath (ComputeShapeVisitor::_localNodePath) that contains all Transform nodes encountered during traversal
    except AbsoluteModelTransform nodes. */
    void apply( osg::Transform& node );
    /** \brief Creates a btCollisionShape for the Geode and its Drawables.

    Obtains the local-to-world matrix from the current ComputeShapeVisitor::_localNodePath, then
    calls createAndAddShape(). */
    void apply( osg::Geode& node );

    /** After the visitor has traversed the scene graph, call this function to
    obtain a pointer to the created collision shape. The calling code is responsible
    for deleting the btCollisionShape pointer to avoid memory leaks. */
    btCollisionShape* getShape();
    /** \overload */
    const btCollisionShape* getShape() const;

protected:
    /** \brief Calls createShape() and adds the result to the master btCompoundShape.
    */
    void createAndAddShape( osg::Node& node, const osg::Matrix& m );

    /** \brief Creates a btCollisionShape for the specified Node.

    Creates a deep copy of the Geode, transforms the copy, and creates a
    collision shape from the transformed data.

    If ComputeShapeVisitor::_shapeType speciies a box, sphere, or cylinder, this function adds the
    created shape as a child to a btCompoundShape, with a btTransform that accounts
    for the bounding volume center. btCompoundShape is not used if ComputeShapeVisitor::_shapeType is a
    triangle mesh, convex tri mesh, or convex hull, as the shape is already in
    the appropriate coordinate space.

    \param node Currently, this must be a Geode.
    \param m Accumulated transformation of the \c node. This function creates
    a deep copy of \c node, transforms the copy by \c m, creates the collision
    shape from the transformed copy, then discards the copy.
    \return NULL if \c node is not a Geode, or if ComputeShapeVisitor::_shapeType is unsupported.
    Otherwise, returns the created collision shape.
    */
    btCollisionShape* createShape( osg::Node& node, const osg::Matrix& m );

    /** Called when _shapeType indicates a triangle mesh or convex triangle mesh.
    Uses _reductionLevel to reduce triangle count in the input subgraph. */
    void reduce( osg::Node& node );

    /** Shape to create per Geode. Set in ComputeShapeVisitor(). */
    const BroadphaseNativeTypes _shapeType;
    /** If _shapeType is a cylinder, created shapes use this axis. Set in ComputeShapeVisitor(). */
    const osgbCollision::AXIS _axis;

    /** If _shapeType is a triangle mesh or convex triangle mesh, geometry is reduced prior to creating
    the collision shape. Range is 0 (no reduction) to 3 (aggressive reduction). */
    const unsigned int _reductionLevel;
    /** Computed in the first invoked apply() method to compute the overall bounding volume.
    Used in geometry reduction if _reductionLevel is greater than zero and _shapeType indicates
    a triangle mesh or convex triangle mesh. */
    osg::BoundingSphere _bs;

    /** This is the created collision shape representing the traversed scene graph.
    Obtain it by calling getShape(). */
    btCollisionShape* _shape;

    /** NodePath containing only Transform nodes, but excluding AbsoluteModelTransform. */
    osg::NodePath _localNodePath;
};


// osgbCollision
}


// __OSGCOLLISION_COMPUTE_SHAPE_VISITOR_H__
#endif
