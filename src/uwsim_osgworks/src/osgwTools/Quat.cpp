/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgWorks is (C) Copyright 2009-2012 by Kenneth Mark Bryden
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

#include "osgwTools/Quat.h"
#include <osg/Quat>
#include <osg/Matrix>
#include <osg/Vec3>

#include <osg/Notify>


namespace osgwTools
{

osg::Quat
makeHPRQuat( osg::Vec3 rotAngles )
{
    return( makeHPRQuat( rotAngles[ 0 ], rotAngles[ 1 ], rotAngles[ 2 ] ) );
}

osg::Quat
makeHPRQuat( double h, double p, double r )
{
    OSG_NOTICE << "makeHPRQuat() is deprecated. Use Orientation instead." << std::endl;


    // Given h, p, and r angles in degrees, build a Quat to affect these rotatiions.
    // We do this by creating a Matrix that contains correctly-oriented x, y, and
    // z axes. Then we create the Quat from the Matrix.
    //
    // First, create x, y, and z axes that represent the h, p, and r angles.
    //   Rotate x and y axes by the heading.
    osg::Vec3 z( 0., 0., 1. );
    osg::Quat qHeading( osg::DegreesToRadians( h ), z );
    osg::Vec3 x = qHeading * osg::Vec3( 1., 0., 0. );
    osg::Vec3 y = qHeading * osg::Vec3( 0., 1., 0. );
    //   Rotate z and y axes by the pitch.
    osg::Quat qPitch( osg::DegreesToRadians( p ), x );
    y = qPitch * y;
    z = qPitch * z;
    //   Rotate x and z axes by the roll.
    osg::Quat qRoll( osg::DegreesToRadians( r ), y );
    x = qRoll * x;
    z = qRoll * z;
    // Use x, y, and z axes to create an orientation matrix.
    osg::Matrix m( x[0], x[1], x[2], 0.,
                  y[0], y[1], y[2], 0.,
                  z[0], z[1], z[2], 0.,
                  0., 0., 0., 1. );

    osg::Quat quat;
    quat.set( m );
    return( quat );
}



// namespace osgwTools
}
