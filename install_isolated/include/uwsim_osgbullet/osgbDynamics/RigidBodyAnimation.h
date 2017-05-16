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

#ifndef __OSGBDYAMICS_RIGID_BODY_ANIMATION_H__
#define __OSGBDYAMICS_RIGID_BODY_ANIMATION_H__ 1

#include <osg/NodeCallback>

#include <osgbDynamics/Export.h>

namespace osgbDynamics {


/** \class RigidBodyAnimation RigidBodyAnimation.h <osgbDynamics/RigidBodyAnimation.h>
\brief An update callback to reposition a btRigidBody in the Bullet simulation.

This callback repositions an object within the Bullet simulation. Attach it as
an update callback to an OSG MatrixTransform. The MatrixTransform must have an
osgbCollision::RefBulletObject< btRigidBody > attached as UserData. */
class OSGBDYNAMICS_EXPORT RigidBodyAnimation : public osg::NodeCallback
{
public:
    RigidBodyAnimation( void );

    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv );

protected:
    virtual ~RigidBodyAnimation() { }
};


// osgbDynamics
}

// __OSGBDYAMICS_RIGID_BODY_ANIMATION_H__
#endif
