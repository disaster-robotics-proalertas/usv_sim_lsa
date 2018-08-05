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

#include <osgbDynamics/RigidBodyAnimation.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbCollision/Utils.h>
#include <btBulletDynamicsCommon.h>
#include <osg/MatrixTransform>

#include <osg/Notify>
#include <osg/io_utils>

#include <iostream>

#include <btBulletCollisionCommon.h>


namespace osgbDynamics
{


RigidBodyAnimation::RigidBodyAnimation( void )
{
}

void RigidBodyAnimation::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
    osg::MatrixTransform* matTrans = dynamic_cast< osg::MatrixTransform* >( node );
    if( node == NULL )
    {
        return;
    }

    osgbCollision::RefBulletObject< btRigidBody >* rb = dynamic_cast<
        osgbCollision::RefBulletObject< btRigidBody >* >( matTrans->getUserData() );
    if( rb == NULL )
    {
        osg::notify( osg::WARN ) << "RigidBodyAnimation requires RefBulletObject<btRigidBody> in Node::UserData." << std::endl;
        return;
    }

    btRigidBody* body = rb->get();
    if( body->getInvMass() != 0.0 )
    {
        return;
    }

    osg::Matrix mat = matTrans->getMatrix();
    rb->get()->getMotionState()->setWorldTransform(
        osgbCollision::asBtTransform( mat ) );

    traverse( node, nv );
}


// osgbDynamics
}
