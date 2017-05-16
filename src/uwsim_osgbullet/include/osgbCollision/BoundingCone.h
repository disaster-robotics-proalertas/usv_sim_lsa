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

#ifndef __OSGBCOLLISION_BOUNDINGCONE_H__
#define __OSGBCOLLISION_BOUNDINGCONE_H__ 1


#include <osgbCollision/Export.h>
#include <osg/Vec3>


namespace osgbCollision
{


/** \class BoundingCone BoundingCone.h <osgbCollision/BoundingCone.h>
\brief Used internally to store cone parameters.

TBD Not currently used. when we do support cones, consider not encapsulating parameters in a class. */
class OSGBCOLLISION_EXPORT BoundingCone
{
public:
    BoundingCone( void );
    virtual ~BoundingCone( void );

    void init()
    {
        length = radius = 0.0f;
    }

    void setAxis( const osg::Vec3 & a )
    {
        axis = a;
        axis.normalize();
    }
    const osg::Vec3 & getAxis() const
    {
        return( axis );
    }

    void setRadius( float r )
    {
        radius = r;
    }
    float getRadius() const
    {
        return( radius );
    }

    void setLength( float l )
    {
        length = l;
    }
    float getLength() const
    {
        return( length );
    }

    void expandBy( const osg::Vec3& v );


    void expandBy( float x,
                   float y,
                   float z );

    void expandBy( const BoundingCone& bc );

protected:
    float length;
    float radius;
    osg::Vec3 axis;
};


// osgbCollision
}


// __OSGBCOLLISION_BOUNDINGCONE_H__
#endif
