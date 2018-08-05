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

#include <osgbDynamics/RigidBody.h>
#include <osgbDynamics/CreationRecord.h>
#include <osgbDynamics/MotionState.h>
#include <osgbCollision/CollisionShapes.h>

#include <osg/Node>
#include <osg/MatrixTransform>
#include <osg/BoundingSphere>
#include <osg/Notify>
#include <osg/ref_ptr>
#include <osg/io_utils>


namespace osgbDynamics
{


btRigidBody* createRigidBody( osgbDynamics::CreationRecord* cr )
{
    osg::Node* root = cr->_sceneGraph;
    if( root == NULL )
    {
        osg::notify( osg::WARN ) << "createRigidBody: CreationRecord has NULL scene graph." << std::endl;
        return( NULL );
    }

    osg::BoundingSphere bs = root->getBound();


    // Bullet collision shapes must be centered on the origin for correct
    // center of mass behavior. Calling code should call
    // CreationRecord::setCenterOfMass() to specify COM. Otherwise, this
    // function uses the bounding volume center as the COM.
    // Translate this subgraph so it is centered on the COM.
    osg::notify( osg::DEBUG_FP ) << "createRigidBody: ";
    osg::Vec3 com;
    if( cr->_comSet )
    {
        // Use user-specified center of mass.
        com = cr->_com;
        osg::notify( osg::DEBUG_FP ) << "User-defined ";
    }
    else
    {
        // Compute from bounding sphere.
        com = bs.center();
        osg::notify( osg::DEBUG_FP ) << "Bounding sphere ";
    }
    osg::notify( osg::DEBUG_FP ) << "center of mass: " << com << std::endl;

    // Create a temporary Transform node containing the center of mass offset and scale vector.
    // Use this as the root of the scene graph for conversion to a collision shape.
    osg::Matrix m( osg::Matrix::translate( -com ) * osg::Matrix::scale( cr->_scale ) );
    osg::ref_ptr< osg::MatrixTransform > tempMtRoot = new osg::MatrixTransform( m );
    tempMtRoot->addChild( root );


    osg::notify( osg::DEBUG_FP ) << "createRigidBody: Creating collision shape." << std::endl;
    btCollisionShape* shape( NULL );
    if( cr->_overall )
    {
        switch( cr->_shapeType )
        {
        case BOX_SHAPE_PROXYTYPE:
            shape = osgbCollision::btCompoundShapeFromBounds( tempMtRoot.get(), BOX_SHAPE_PROXYTYPE );
            break;
        case SPHERE_SHAPE_PROXYTYPE:
            shape = osgbCollision::btCompoundShapeFromBounds( tempMtRoot.get(), SPHERE_SHAPE_PROXYTYPE );
            break;
        case CYLINDER_SHAPE_PROXYTYPE:
            shape = osgbCollision::btCompoundShapeFromBounds( tempMtRoot.get(), CYLINDER_SHAPE_PROXYTYPE, cr->_axis );
            break;
        case TRIANGLE_MESH_SHAPE_PROXYTYPE:
            shape = osgbCollision::btTriMeshCollisionShapeFromOSG( tempMtRoot.get() );
            break;
        case CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE:
        {
            btConvexTriangleMeshShape* cShape = osgbCollision::btConvexTriMeshCollisionShapeFromOSG( tempMtRoot.get() );
            if( cr->_marginSet )
                cShape->setMargin( cr->_margin );
            shape = cShape;
            break;
        }
        case CONVEX_HULL_SHAPE_PROXYTYPE:
        {
            btConvexHullShape* cShape = osgbCollision::btConvexHullCollisionShapeFromOSG( tempMtRoot.get() );
            if( cr->_marginSet )
                cShape->setMargin( cr->_margin );
            shape = cShape;
            break;
        }
        }
    }
    else
    {
        shape = osgbCollision::btCompoundShapeFromOSGGeodes( tempMtRoot.get(),
            cr->_shapeType, cr->_axis, static_cast< unsigned int >( cr->_reductionLevel ) );
    }
    if( shape == NULL )
    {
        osg::notify( osg::WARN ) << "createRigidBody: btCompoundShapeFromOSGGeodes returned NULL." << std::endl;
        return( NULL );
    }

    return( createRigidBody( cr, shape ) );
}

btRigidBody* createRigidBody( osgbDynamics::CreationRecord* cr, btCollisionShape* shape )
{
    osg::Node* root = cr->_sceneGraph;
    if( root == NULL )
    {
        osg::notify( osg::WARN ) << "createRigidBody: CreationRecord has NULL scene graph." << std::endl;
        return( NULL );
    }


    osg::notify( osg::DEBUG_FP ) << "createRigidBody: Creating rigid body." << std::endl;
	btVector3 localInertia( 0, 0, 0 );
    const bool isDynamic = ( cr->_mass != 0.f );
	if( isDynamic )
		shape->calculateLocalInertia( cr->_mass, localInertia );

    // Create MotionState to control OSG subgraph visual reprentation transform
    // from a Bullet world transform. To do this, the MotionState need the address
    // of the Transform node (must be either AbsoluteModelTransform or
    // MatrixTransform), center of mass, scale vector, and the parent (or initial)
    // transform (usually the non-scaled OSG local-to-world matrix obtained from
    // the parent node path).
    osgbDynamics::MotionState* motion = new osgbDynamics::MotionState();
    osg::Transform* trans = dynamic_cast< osg::Transform* >( root );
    if( trans != NULL )
        motion->setTransform( trans );

    osg::Vec3 com;
    if( cr->_comSet )
        com = cr->_com;
    else
        com = root->getBound().center();
    motion->setCenterOfMass( com );

    motion->setScale( cr->_scale );
    motion->setParentTransform( cr->_parentTransform );

    // Finally, create rigid body.
    btRigidBody::btRigidBodyConstructionInfo rbInfo( cr->_mass, motion, shape, localInertia );
    rbInfo.m_friction = btScalar( cr->_friction );
    rbInfo.m_restitution = btScalar( cr->_restitution );
	btRigidBody* rb = new btRigidBody( rbInfo );
    if( rb == NULL )
    {
        osg::notify( osg::WARN ) << "createRigidBody: Created a NULL btRigidBody." << std::endl;
        return( NULL );
    }

    // Last thing to do: Position the rigid body in the world coordinate system. The
    // MotionState has the initial (parent) transform, and also knows how to account
    // for center of mass and scaling. Get the world transform from the MotionState,
    // then set it on the rigid body, which in turn sets the world transform on the
    // MotionState, which in turn transforms the OSG subgraph visual representation.
    btTransform wt;
    motion->getWorldTransform( wt );
    rb->setWorldTransform( wt );

    return( rb );
}


// osgbDynamics
}
