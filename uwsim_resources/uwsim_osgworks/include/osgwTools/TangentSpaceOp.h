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

#ifndef __OSGWTOOLS_TANGENT_SPACE_OP_H__
#define __OSGWTOOLS_TANGENT_SPACE_OP_H__ 1


#include <osgwTools/Export.h>
#include <osgwTools/GeometryOperation.h>
#include <osgwTools/TangentSpaceGeneratorDouble.h>
#include <osg/CopyOp>
#include <osg/Object>
#include <osg/Vec3>

namespace osgwTools {


/** \class TangentSpaceOp TangentSpaceOp.h <osgwTools/TangentSpaceOp.h>
\brief Runs TangentSpaceGeneratorDouble on all Geometry objects.
*/
class OSGWTOOLS_EXPORT TangentSpaceOp : public GeometryOperation
{
public:
    TangentSpaceOp();
    TangentSpaceOp( const unsigned int normalMapTextureUnit, const unsigned int tangentIndex=6, const unsigned int binormalIndex=7, const unsigned int normalIndex=15 );
    TangentSpaceOp( const TangentSpaceOp& rhs, const osg::CopyOp& copyOp=osg::CopyOp::SHALLOW_COPY );
    ~TangentSpaceOp();

    META_Object(osgwTools,TangentSpaceOp);

    virtual osg::Geometry* operator()( osg::Geometry& geom );

    void setNormalMapTextureUnit( const unsigned int normalMapTextureUnit );
    unsigned int getNormalMapTextureUnit() const;

    void setVertexArrayLocations( const unsigned int tangentIndex=6, const unsigned int binormalIndex=7, const unsigned int normalIndex=15 );
    void getVertexArrayLocations( unsigned int& tangentIndex, unsigned int& binormalIndex, unsigned int& normalIndex );

protected:
    unsigned int _normalMapTextureUnit;
    unsigned int _tangentIndex;
    unsigned int _binormalIndex;
    unsigned int _normalIndex;

    osg::ref_ptr< TangentSpaceGeneratorDouble > _tsg;
};


// namespace osgwTools
}

// __OSGWTOOLS_TANGENT_SPACE_OP_H__
#endif
