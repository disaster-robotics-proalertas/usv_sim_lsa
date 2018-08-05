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

#include <osgbCollision/BoundingCylinder.h>
#include <osg/io_utils>
#include <iostream>


namespace osgbCollision
{


BoundingCylinder::BoundingCylinder( void )
{
    axis = osg::Vec3( 0, 1, 0 );
    length = 0.0;
    radius = 0.0;
}

BoundingCylinder::~BoundingCylinder( void )
{
}

void BoundingCylinder::expandBy( const osg::Vec3& v )
{
    float nl, nr;

    nl = osg::absolute( v * axis );
    if( nl > length )
    {
        length = nl;
    }

    nr = sqrtf( v.length2() - nl * nl );
    if( nr > radius )
    {
        radius = nr;
    }
}

void BoundingCylinder::expandBy( float x,
                                 float y,
                                 float z )
{
    expandBy( osg::Vec3( x, y, z ) );
}

void BoundingCylinder::expandBy( const BoundingCylinder& bc )
{
    float a, b;

    a = osg::absolute( bc.getAxis() * axis );
    b = sqrtf( 1 - a * a );

    float nl = a * bc.getLength() + b * bc.getRadius();
    float nr = sqrtf( b * b * bc.getLength() * bc.getLength() + bc.getRadius() * bc.getRadius() );

    if( nl > length )
    {
        length = nl;
    }
    if( nr > radius )
    {
        radius = nr;
    }

    return;
}


// osgbCollision
}
