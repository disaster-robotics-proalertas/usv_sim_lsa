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

#include <osgbCollision/BoundingCone.h>
#include <osg/io_utils>
#include <iostream>


namespace osgbCollision
{


BoundingCone::BoundingCone( void )
{
    axis = osg::Vec3( 0, 1, 0 );
    length = 0.0;
    radius = 0.0;
}

BoundingCone::~BoundingCone( void )
{
}

void BoundingCone::expandBy( const osg::Vec3& v )
{
    float nl, nr;

    nl = v * axis;
    if( ( nl < 0.0 ) && ( nl < -length ) )
    {
        length = nl;
    }

    nr = sqrtf( v.length2() - nl * nl );
    if( nr > radius )
    {
        radius = nr;
    }
}

void BoundingCone::expandBy( float x,
                             float y,
                             float z )
{
    expandBy( osg::Vec3( x, y, z ) );
}

void BoundingCone::expandBy( const BoundingCone& bc )
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
