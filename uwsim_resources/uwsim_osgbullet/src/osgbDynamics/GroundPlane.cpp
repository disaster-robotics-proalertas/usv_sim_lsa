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

#include <osgbDynamics/GroundPlane.h>
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <osgbCollision/Utils.h>

#include <osg/Geometry>
#include <osg/Geode>

#include <osg/io_utils>
#include <iostream>


namespace osgbDynamics
{


osg::Node* generateGroundPlane( const osg::Vec4& plane, btDynamicsWorld* bulletWorld, btRigidBody** rb, const short group, const short mask )
{
    osg::Vec3 n(plane.x(),plane.y(),plane.z());
    n.normalize();
    float d (plane.w());
    osg::Vec3 v (1.f,0,0);

    // TBD consider using osg::Vec3::operator^ for cross product: (v^n)
    osg::Vec3 u1 = v -n*(v.x()*n.x() +v.y()*n.y() + v.z()*n.z());
    osg::Vec3 u2;
    if (u1.length()==0){
        u1 = osg::Vec3(0.f,1.f,0.f);
        u2 = osg::Vec3(0.f,0.f,1.f);
    }
    else{
        u1.normalize();
        u2 = n^u1;
        u2.normalize();
    }

    osg::Vec3 p =  n * d;

    // TBD use new stuff in Shapes.
    const btVector3 planeNormal( plane.x(), plane.y(), plane.z() );
    btCollisionShape* groundShape = new btStaticPlaneShape( planeNormal, plane.w() );
    btRigidBody::btRigidBodyConstructionInfo rbInfo( 0., NULL, groundShape, btVector3(0,0,0) );
    btRigidBody* ground = new btRigidBody(rbInfo);

    btDiscreteDynamicsWorld* dw = dynamic_cast< btDiscreteDynamicsWorld* >( bulletWorld );
    if( ( dw != NULL ) && ( ( group != 0 ) || ( mask != 0 ) ) )
        dw->addRigidBody( ground, group, mask );
    else
        bulletWorld->addRigidBody( ground );

    if( rb != NULL )
        *rb = ground;

    osg::ref_ptr< osg::Geode > groundPlane = new osg::Geode;
    osg::Geometry* groundGeom = new osg::Geometry;
    groundPlane->addDrawable(groundGeom);
    osg::ref_ptr<osg::Vec3Array> vertarray = new osg::Vec3Array;
    groundGeom->setVertexArray( vertarray.get() );

    int width(30);
    osg::Vec3 point;
    const int nVerts( 4*width+2 );
    for(int i = -width; i < width; i++)
    {
        for(int j = -width; j < width+1; j ++)
        {  
            vertarray->push_back(p + u1*i + u2*j);
            vertarray->push_back(p + u1*(i+1) + u2*j);
        }
        groundGeom->addPrimitiveSet( new osg::DrawArrays(
            osg::PrimitiveSet::TRIANGLE_STRIP, (i+width)*nVerts, nVerts ) );
    }

    osg::ref_ptr<osg::Vec3Array> norm = new osg::Vec3Array;
    groundGeom->setNormalArray( norm.get() );
    norm->push_back( n );
    groundGeom->setNormalBinding( osg::Geometry::BIND_OVERALL );

    osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array;
    groundGeom->setColorArray( c.get() );
    c->push_back( osg::Vec4( 1.f, 1.f, 1.f, 1.f ) );
    groundGeom->setColorBinding( osg::Geometry::BIND_OVERALL );

    return( groundPlane.release() );
}



// osgbDynamics
}
