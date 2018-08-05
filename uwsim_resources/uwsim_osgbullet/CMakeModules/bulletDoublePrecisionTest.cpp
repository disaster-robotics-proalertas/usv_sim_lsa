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

/*
This file is intended to be used by the CMake try_compile command.
When built with -DBT_BULLET_DOUBLE_PRECISION, the compile/link will
succeed only if Bullet was built with the same definition, in which
case osgBullet's CMake system will set the corresponding
-DBT_BULLET_DOUBLE_PRECISION definition for the osgBullet project.
If Bullet was not built with -DBT_BULLET_DOUBLE_PRECISION, the build
fails (btStaticPlaneShape comes up as an undefined symbol because
btVector3 uses a different base type) and osgBullet will be build
for single precision physics.
*/


#include <btBulletCollisionCommon.h>

int main()
{
    btVector3 vec( 0., 1., 0. );
    btStaticPlaneShape* bsps = new btStaticPlaneShape( vec, 0. );

    return( 0 );
}
