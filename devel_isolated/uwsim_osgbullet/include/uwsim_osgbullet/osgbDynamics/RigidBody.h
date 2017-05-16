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

#ifndef __OSGBDYNAMICS_RIGID_BODY_H__
#define __OSGBDYNAMICS_RIGID_BODY_H__ 1

#include <osgbDynamics/Export.h>
#include <osgbDynamics/CreationRecord.h>
#include <btBulletDynamicsCommon.h>


namespace osgbDynamics
{


/** \defgroup rigidbody Rigid Body Creation
\brief Convenience routines for creating Bullet rigid bodies from scene graphs.

These functions use the CreationRecord struct to create collision shapes,
rigid bodies, and motion state objects. The application is responsible for
setting the CreationRecord fields as needed. The fields are used as follows:

<ul>
  <li> \c _sceneGraph Geometric source data for collision shape and rigid body creation.
    <ul>
      <li>Passed to osgbCollision::btCompoundShapeFromOSGGeodes() to create a collision shape.</li>
      <li>If \c _comSet is false, the \c _sceneGraph bounding sphere center is used as the center of mass.</li>
      <li>If \c _sceneGraph is a Transform node, it is set as the managed Transform node in MotionState (MotionState::setTransform()).</li>
    </ul>
  </li>
  <li> \c _com When \c _comSet is true, \c _com is the center of mass. Otherwise, the bounding volume
       center is used as the center of mass. (See CreationRecord::setCenterOfMass().)
    <ul>
      <li>The negated center of mass multiplied by the \c _scale vector is used as a transform for geometric data during collision shape creation.
      <li>The center of mass is passed to the created MotionState object (MotionState::setCenterOfMass()).
    </ul>
  </li>
  <li> \c _scale \em xyz scale vector. (osgBullet supports non-uniform scaling.)
    <ul>
      <li>The negated center of mass multiplied by the \c _scale vector is used as a transform for geometric data during collision shape creation.
      <li>\c _scale is passed to the created MotionState object (MotionState::setScale()).
    </ul>
  </li>
  <li> \c _parentTransform Used to specify an initial position. It is set as the MotionState parent transform (MotionState::setParentTransform()).
  </li>
  <li> \c _shapeType Passed to osgbCollision::btCompoundShapeFromOSGGeodes().
  </li>
  <li> \c _mass
    <ul>
      <li>Passed to btCollisionShape::calculateLocalInertia().
      <li>Set in the created rigid body via btRigidBody::btRigidBodyConstructionInfo.
    </ul>
  </li>
  <li> \c _restitution Set in the created rigid body via btRigidBody::btRigidBodyConstructionInfo::m_restitution.
  </li>
  <li> \c _friction Set in the created rigid body via  btRigidBody::btRigidBodyConstructionInfo::m_friction.
  </li>
  <li> \c _axis Passed to osgbCollision::btCompoundShapeFromOSGGeodes().
    Ultimately, it is referenced only if \c _shapeType is \c CYLINDER_SHAPE_PROXYTYPE.
  </li>
  <li> \c _reductionLevel Passed to osgbCollision::btCompoundShapeFromOSGGeodes().
    If \c _shapeType is \c TRIANGLE_MESH_SHAPE_PROXYTYPE or CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE,
    this value is used to configure osgWorks geometry reduction utilities to reduce triangle
    count prior to creating the collision shape.
  </li>
</ul>

Note that these are merely convenience routines, and your application can interface
directly with Bullet to create collision shapes and rigid bodies. However, you should
strongly consider using these convenience routines for the following reasons:

\li The routines are based on the CreationRecord class, which supports the dot OSG
file format. This allows your application to save and restore creation paramters.
\li These routines support non-origin center of mass.
\li These routines support scaled collision shapes.
\li These routines support initial transformations (for example, from the
accumulated OSG local-to-world matrix in the subgraph parent's NodePath).
\li These routines automatically create a MotionState object to keep the OSG visual
representation in sync with the Bullet physics representation.

<b>Requirements</b>

\li The root node of the subgraph (CreationRecord::_sceneGraph) must be either an
osg::MatrixTransform or an osgwTools::AbsoluteModelTransform (from the osgWorks project).
Other osg::Transform-derived nodes are not supported.

<b>Functionality Removed in v2.0</b>

\li The <em>overall</em> feature created a single collision shape around the entire
subgraph. You can still obtain this same functionality by calling the
\link collisionshapes collision shape functions \endlink directly, then
passing the created collision shape to createRigidBody().
\li The <em>named node</em> feature searched the subgraph for the named node, then
used that node as the basis for creating the collision shape. The application is
now responsible for doing this work. The FindNamedNode visitor in osgWorks can be
used to find nodes with a specific name.

*/
/*@{*/


/** \brief Creates a compound collision shape and rigid body from the CreationRecord data.

Uses the osgbCollision::ComputeShapeVisitor to create a btCompoundShape from CreationRecord::_sceneGraph.
Currently, a shape per Geode is created. CreationRecord::_shapeType specifies the shape type created per Geode.
If CreationRecord::_shapeType is CYLINDER_SHAPE_PROXYTYPE, CreationRecord::_axis specifies the cylinder major axis.
*/
OSGBDYNAMICS_EXPORT btRigidBody* createRigidBody( osgbDynamics::CreationRecord* cr );

/** \overload
<p>
Use this function to create a rigid body if you have already created the collision shape.
This is useful for sharing collision shapes.
</p>
*/
OSGBDYNAMICS_EXPORT btRigidBody* createRigidBody( osgbDynamics::CreationRecord* cr, btCollisionShape* shape );


/**@}*/


// osgbDynamics
}


// __OSGBDYNAMICS_RIGID_BODY_H__
#endif
