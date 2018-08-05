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

#include <osgbInteraction/GestureHandler.h>
#include <osgbInteraction/HandNode.h>
#include <osg/Notify>

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>


namespace osgbInteraction
{


// Static gesture codes
const unsigned int GestureHandler::Unknown( 0 );
const unsigned int GestureHandler::Default( 1 );
const unsigned int GestureHandler::Point( 2 );
const unsigned int GestureHandler::Fist( 3 );


GripRelease::GripRelease()
  : _constraint( NULL )
{
}
GripRelease::~GripRelease()
{
    if( _constraint != NULL )
        delete _constraint;
}

bool
GripRelease::operator()( const unsigned int gestureCode, HandNode& handNode )
{
    switch( gestureCode ) {
    case Default:
    {
        osg::notify( osg::INFO ) << "Received Default." << std::endl;

        btDynamicsWorld* bulletWorld = handNode.getDynamicsWorld();
        if( _constraint != NULL )
        {
            bulletWorld->removeConstraint( _constraint );
            _constraint = NULL;
        }

        return( true );
        break;
    }
    case Fist:
    {
        osg::notify( osg::INFO ) << "Received Fist." << std::endl;

        // Physics thread should already be stopped; see HandNode::sendGestureCode.
        btRigidBody* closest = handNode.findClosest();
        if( closest == NULL )
        {
            osg::notify( osg::WARN ) << "GripRelease got NULL from HandNode::findClosest()." << std::endl;
            return( false );
        }

        // Constraint parameters: The transform for the selected object in
        // the hand's coordinate space is: HandWTInv * BodyWT.
        btTransform xformA = handNode.getRigidBody()->getWorldTransform().inverse() *
            closest->getWorldTransform();
        btTransform xformB;
        xformB.setIdentity();

        // Constrain.
        _constraint = new btGeneric6DofConstraint( *( handNode.getRigidBody() ),
            *closest, xformA, xformB, false );
        // Constraint rotations.
        _constraint->setAngularLowerLimit( btVector3(0,0,0) );
        _constraint->setAngularUpperLimit( btVector3(0,0,0) );
        handNode.getDynamicsWorld()->addConstraint( _constraint, true );

        return( true );
        break;
    }
    default:
    {
        osg::notify( osg::ALWAYS ) << "Unknown gesture code: " << gestureCode << std::endl;
        return( false );
        break;
    }
    }
}


// osgbInteraction
}
