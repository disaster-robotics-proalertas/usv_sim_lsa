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

#ifndef __OSGBDYNAMICS_EXPORT_H__
#define __OSGBDYNAMICS_EXPORT_H__ 1


#if defined( _MSC_VER ) || defined( __CYGWIN__ ) || defined( __MINGW32__ ) || defined( __BCPLUSPLUS__ ) || defined( __MWERKS__ )
    #if defined( OSGBULLET_STATIC )
        #define OSGBDYNAMICS_EXPORT
    #elif defined( OSGBULLET_SHARED ) && defined( OSGBDYNAMICS_LIBRARY )
        #define OSGBDYNAMICS_EXPORT __declspec( dllexport )
    #else
        #define OSGBDYNAMICS_EXPORT __declspec( dllimport )
    #endif
#else
    #define OSGBDYNAMICS_EXPORT
#endif


// __OSGBDYNAMICS_EXPORT_H__
#endif
