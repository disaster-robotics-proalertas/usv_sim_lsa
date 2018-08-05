/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgBullet is (C) Copyright 2009-2011 by Kenneth Mark Bryden
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

/** \page bulletprecisiontest Bullet Precision Test

This test program determins whether Bullet was compiled with single
or double precision. The return code value is 1 for single precision
and 2 for double precision. 0 is returned for an error.

Build this as a standalone Bullet application to determine whether
Bullet was built with single or double precision, then use the output
to configure your external application or project.

Originally contributed by Kevin Godby. */

#include <LinearMath/btScalar.h>

#include <typeinfo>
#include <iostream>

int main( int argc, char **argv )
{
    int returnCode( 0 );
    std::cout << "Bullet version: " << BT_BULLET_VERSION;
    if(typeid( btScalar ) == typeid( double ) )
    {
        std::cout << " (double precision)." << std::endl;
        returnCode = 2;
    }
    else if( typeid( btScalar ) == typeid( float ) )
    {
        std::cout << " (single precision)." << std::endl;
        returnCode = 1;
    }
    else
    {
        std::cout << std::endl;
        std::cerr << "ERROR: Type of btScalar is [" << typeid( btScalar ).name() << "]." << std::endl;
        returnCode = 0;
    }

    std::cout << "  Returning code " << returnCode << std::endl;
    return( returnCode );
}

