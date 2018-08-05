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

#ifndef __OSGBULLET_GROUND_PLANE_H__
#define __OSGBULLET_GROUND_PLANE_H__ 1


#include <osgbDynamics/Export.h>
#include <btBulletDynamicsCommon.h>
#include <osg/Vec4>


namespace osg {
    class Node;
}

namespace osgbDynamics
{


/** \brief Add a plane rigid body to the dynamics world and return an OSG subgraph to render the plane.
*/
OSGBDYNAMICS_EXPORT osg::Node* generateGroundPlane( const osg::Vec4& plane, btDynamicsWorld* bulletWorld, btRigidBody** rb=NULL, short group=0, short mask=0 );


// osgbDynamics
}


// __OSGBULLET_GROUND_PLANE_H__
#endif
