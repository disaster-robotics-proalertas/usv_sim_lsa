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

// This code was taken from OSG v2.8.5 and modified to use double precision.
// Original copyright:

/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield 
 *
 * This library is open source and may be redistributed and/or modified under  
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or 
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * OpenSceneGraph Public License for more details.
*/

#ifndef __OSGWTOOLS_TANGENT_SPACE_GENERATOR_DOUBLE_H__
#define __OSGWTOOLS_TANGENT_SPACE_GENERATOR_DOUBLE_H__ 1

#include <osgwTools/Export.h>

#include <osg/ref_ptr>
#include <osg/Referenced>
#include <osg/Array>
#include <osg/Geometry>


namespace osgwTools
{

/** \class TangentSpaceGeneratorDouble TangentSpaceGeneratorDouble.h <osgwTools/TangentSpaceGeneratorDouble.h>
\brief Double precision verstion of the osgUtil TangentSpaceGenerator.
\details This class is identical to osgUtil::TangentSpaceGenerator, except
the generate() and compute() member functions have been modified to perform
all computations using double rather than single precision.

TangentSpaceGeneratorDouble is based on the OSG v285 TangentSpaceGenerator source.
*/
class OSGWTOOLS_EXPORT TangentSpaceGeneratorDouble: public osg::Referenced
{
public:
    TangentSpaceGeneratorDouble();
    TangentSpaceGeneratorDouble(const TangentSpaceGeneratorDouble &copy, const osg::CopyOp &copyop = osg::CopyOp::SHALLOW_COPY);

    void generate(osg::Geometry *geo, int normal_map_tex_unit = 0);

    inline osg::Vec4Array *getTangentArray()               { return T_.get(); }
    inline const osg::Vec4Array *getTangentArray() const   { return T_.get(); }
    inline void setTangentArray(osg::Vec4Array *array)     { T_ = array; }

    inline osg::Vec4Array *getNormalArray()                { return N_.get(); }
    inline const osg::Vec4Array *getNormalArray() const    { return N_.get(); }
    inline void setNormalArray(osg::Vec4Array *array)      { N_ = array; }

    inline osg::Vec4Array *getBinormalArray()              { return B_.get(); }
    inline const osg::Vec4Array *getBinormalArray() const  { return B_.get(); }
    inline void setBinormalArray(osg::Vec4Array *array)    { B_ = array; }

    inline osg::IndexArray *getIndices() { return indices_.get(); }

protected:

    virtual ~TangentSpaceGeneratorDouble() {}
    TangentSpaceGeneratorDouble &operator=(const TangentSpaceGeneratorDouble &) { return *this; }

    void compute(osg::PrimitiveSet *pset,
                 const osg::Array *vx,
                 const osg::Array *nx,
                 const osg::Array *tx,
                 int iA, int iB, int iC);

    osg::ref_ptr<osg::Vec4Array> T_;
    osg::ref_ptr<osg::Vec4Array> B_;
    osg::ref_ptr<osg::Vec4Array> N_;
    osg::ref_ptr<osg::UIntArray> indices_;
};


// osgwTools
}


// __OSGWTOOLS_TANGENT_SPACE_GENERATOR_DOUBLE_H__
#endif
