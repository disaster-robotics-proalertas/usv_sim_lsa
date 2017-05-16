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

#ifndef __OSGWTOOLS_QUAT__
#define __OSGWTOOLS_QUAT__ 1


#include "osgwTools/Export.h"
#include <osg/Quat>
#include <osg/Vec3>


namespace osgwTools
{


/** \defgroup Quat Quaternion utilities
*/
/*@{*/

/** Makes a Quat from hpr angles, avoiding gimble lock. It does this by
creating an orthonormal basis of xyz axis vectors and reorienting them
in sequence first by heading, then pitch, then roll.
The hpr angles are in degrees and 
assumes left-handed and z-up, so:
  \li h rotates around the z axis
  \li p rotates aroung the x' axis
  \li r rotates around the y'' axis

\deprecated Please use Orientation instead.
*/
osgwDEPRECATED( OSGWTOOLS_EXPORT osg::Quat makeHPRQuat( double h, double p, double r ) );

/** \overload
\deprecated Please use Orientation instead.
*/
osgwDEPRECATED( OSGWTOOLS_EXPORT osg::Quat makeHPRQuat( osg::Vec3 rotAngles ) );

/*@}*/


// namespace osgwTools
}

// __OSGWTOOLS_INSERT_NODE__
#endif
