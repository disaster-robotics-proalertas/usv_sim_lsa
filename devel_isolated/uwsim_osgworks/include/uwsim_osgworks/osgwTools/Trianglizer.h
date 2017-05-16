/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgWorks is (C) Copyright 2009-2013 by Kenneth Mark Bryden
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

#ifndef __OSGWROOLS_TRIANGLIZER_H__
#define __OSGWROOLS_TRIANGLIZER_H__ 1

#include <osgwTools/Export.h>
#include <osgwTools/GeometryOperation.h>
#include <osg/NodeVisitor>


namespace osgwTools
{


/** \class Trianglizer Trianglizer.h <osgwTools/Trianglizer.h>
\brief Converts surface PrimitiveSets to use GL_TRIANGLES.
\brief For a given Geometry, perform the following operations:
\li Convert GL_TRIANGLE_STRIP, GL_TRIANGLE_FAN, GL_QUADS, and
GL_QUAD_STRIP to use GL_TRIANGLES.
\li Convert DrawArrays, DrawElementsUByte, and DrawElementsUShort,
to DrawElementsUInt.
\li Combine multiple DrawElementsUInt / GL_TRIANGLES PrimitiveSets
into a single DrawElementsUInt / GL_TRIANGLES PrimitiveSet.

The result is the most processed Geometry objects will contain
a single DrawElementsUInt PrimitiveSet configured to render
plain indexed GL_TRIANGES. This appears to be the fastest way
to render geometry in a GL3/4 core profile context on today's
NVIDIA GPUs. */
class OSGWTOOLS_EXPORT Trianglizer : public GeometryOperation
{
public:
    Trianglizer();
    Trianglizer( const Trianglizer& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );

    META_Object(osgwTools,Trianglizer);

    virtual osg::Geometry* operator()( osg::Geometry& geom );

protected:
    virtual ~Trianglizer();

    /** \brief Determines if the given \c geom requires processing.
    Returns true if it requires processing, false otherwise. */
    bool needsConversion( const osg::Geometry& geom );
};


// osgwTools
}


// __OSGWROOLS_TRIANGLIZER_H__
#endif
