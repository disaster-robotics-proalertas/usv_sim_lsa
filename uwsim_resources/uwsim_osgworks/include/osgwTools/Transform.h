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

#ifndef __OSGWTOOLS_TRANSFORM_H__
#define __OSGWTOOLS_TRANSFORM_H__ 1


#include "osgwTools/Export.h"

#include <osg/BoundingSphere>
#include <osg/BoundingBox>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Matrix>


namespace osgwTools
{


/** \defgroup Transform Transformation utilities

\test transform

*/
/*@{*/

/** \brief Transforms a \c BoundingSphere.
*/
OSGWTOOLS_EXPORT osg::BoundingSphere transform( const osg::Matrix& m, const osg::BoundingSphere& sphere );

/** \brief Transforms a \c BoundingBox.
*/
OSGWTOOLS_EXPORT osg::BoundingBox transform( const osg::Matrix& m, const osg::BoundingBox& box );

/** \brief Transforms a \c Geometry in place.

Vertices are transformed by the matrix \c m, and normals are transformed by the upper-left 3x3 portion of \c m.
*/
OSGWTOOLS_EXPORT void transform( const osg::Matrix& m, osg::Geometry* geom );

/** \brief Transforms a \c Vec3Array in place.

Vertices are transformed by the matrix \c m.
*/
OSGWTOOLS_EXPORT void transform( const osg::Matrix& m, osg::Vec3Array* verts, bool normalize=false );

/** \brief Transforms a \c Geode.

Transforms the \c Geometry children of the \c Geode parameter.
*/
OSGWTOOLS_EXPORT void transform( const osg::Matrix& m, osg::Geode* geode );

/*@}*/


// osgwTools
}

// __OSGWTOOLS_GRID_H__
#endif
