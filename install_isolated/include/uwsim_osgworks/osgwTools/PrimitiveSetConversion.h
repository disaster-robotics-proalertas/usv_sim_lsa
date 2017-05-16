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

#ifndef __OSGWTOOLS_PRIMITIVE_SET_CONVERSION_H__
#define __OSGWTOOLS_PRIMITIVE_SET_CONVERSION_H__ 1


#include <osgwTools/Export.h>
#include <osg/Geometry>

namespace osg {
    class DrawArrays;
    class DrawArrayLengths;
    class DrawElementsUByte;
    class DrawElementsUShort;
    class DrawElementsUInt;
}


namespace osgwTools
{


/** \defgroup primconvert PrimitiveSet Conversion Utilities

This set of utility functions performs conversions between various
PrimitiveSet types and primitive modes.

Use these routines to convert
from the (less-efficient) DrawArrays and DrawArrayLengths to
DrawElementsUtint, which can take advantage of the GPU vertex cache.

The convertAllFilledToTriangles() function is useful for converting
all filled primitives to a homogenous GL_TRIANGLES mode, useful for
algorithms that operate on meshes of triangles (such as ReducerOp).
*/
/**@{*/

/** \brief Convert a DrawArrays PrimitiveSet to a DrawElementsUInt.

Creates a new DrawElementsUInt PrimitiveSet with the same mode as \c da,
then stores indices in the DrawElementsUInt to produce the same rendered results as the
oritingl \c da. */
OSGWTOOLS_EXPORT osg::DrawElementsUInt* convertToDEUI( const osg::DrawArrays* da );

/** \brief Convert a DrawArrayLengths PrimitiveSet to a PrimitiveSetList.

Creates one or more DrawElementsUInt PrimitiveSets with the same mode as \c dal and stores
their addresses in the returned PrimitiveSetList. The returned list will render the same
as the original \c dal. */
OSGWTOOLS_EXPORT osg::Geometry::PrimitiveSetList convertToDEUI( const osg::DrawArrayLengths* dal );

/** \brief Convert a DrawElementsUByte PrimitiveSet to a DrawElementsUInt.

This is a simple conversion of the indices to unsigned int. */
OSGWTOOLS_EXPORT osg::DrawElementsUInt* convertToDEUI( const osg::DrawElementsUByte* deub );

/** \overload */
OSGWTOOLS_EXPORT osg::DrawElementsUInt* convertToDEUI( const osg::DrawElementsUShort* deus );

/** \brief Convert any filled primitive mode to GL_TRIANGLES.

If the input \c deul mode is GL_TRIANGLE_STRIP, GL_TRIANGLE_FAN,
GL_QUADS, GL_QUAD_STRIP, or GL_POLYGON, this function returns a
new DrawElementsUInt that uses only GL_TRIANGLES to draw the same
thing as the input \c deui.

If the input \c deul mode is GL_TRIANGLES or any non-filled mode, this
function returns the address of the input \c deui. */
OSGWTOOLS_EXPORT osg::DrawElementsUInt* convertAllFilledToTriangles( const osg::DrawElementsUInt* deuiIn );

/**@}*/

// osgwTools
}


// __OSGWTOOLS_PRIMITIVE_SET_CONVERSION_H__
#endif
