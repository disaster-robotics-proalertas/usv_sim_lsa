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


//
// This code was lifted directly from OSG trunk and ported to osgWorks
// so that it can be used with OSG versions older than v2.9.8.
//


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

#ifndef __OSGWTOOLS_MESH_OPTIMIZERS_H__
#define __OSGWTOOLS_MESH_OPTIMIZERS_H__ 1


#include <osgwTools/Export.h>

#include <set>
#include <vector>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/NodeVisitor>

#include <osgUtil/Optimizer>

namespace osgUtil
{


/** \defgroup MeshOpt Mesh Optimization Tools
\details This is a backport of the osgUtil::MeshOptimizers code module.
It is available in osgWorks so that it can be used with versions of OSG
older than v2.9.8 (for example, v2.8.5), as this is a generally useful
code module and contains almost nothing specific to OSG 2.9.8 and later.

The osgUtil namespace has been preserved to ease porting. Application code
using the mesh optimizers with OSG v2.8.5 and osgWorks, would simply need
to change the header #include when porting to current OSG.
*/
/**\{*/

// Helper that collects all the unique Geometry objects in a subgraph.
class OSGWTOOLS_EXPORT GeometryCollector : public BaseOptimizerVisitor
{
public:
    GeometryCollector(Optimizer* optimizer,
                      Optimizer::OptimizationOptions options)
        : BaseOptimizerVisitor(optimizer, options) {}
    void reset();
    void apply(osg::Geode& geode);
    typedef std::set<osg::Geometry*> GeometryList;
    GeometryList& getGeometryList() { return _geometryList; };
protected:
    GeometryList _geometryList;
};

// Convert geometry that uses DrawArrays to DrawElements i.e.,
// construct a real mesh. This removes duplicate vertices.
class OSGWTOOLS_EXPORT IndexMeshVisitor : public GeometryCollector
{
public:
    IndexMeshVisitor(Optimizer* optimizer = 0)
        : GeometryCollector(optimizer,
            Optimizer::ALL_OPTIMIZATIONS /*Optimizer::INDEX_MESH*/ )
    {
    }
    void makeMesh(osg::Geometry& geom);
    void makeMesh();
};

// Optimize the triangle order in a mesh for best use of the GPU's
// post-transform cache. This uses Tom Forsyth's algorithm described
// at http://home.comcast.net/~tom_forsyth/papers/fast_vert_cache_opt.html
class OSGWTOOLS_EXPORT VertexCacheVisitor : public GeometryCollector
{
public:
    VertexCacheVisitor(Optimizer* optimizer = 0)
        : GeometryCollector(optimizer,
            Optimizer::ALL_OPTIMIZATIONS /*Optimizer::VERTEX_POSTTRANSFORM*/ )
    {
    }

    void optimizeVertices(osg::Geometry& geom);
    void optimizeVertices();
private:
    void doVertexOptimization(osg::Geometry& geom,
                              std::vector<unsigned>& vertDrawList);
};

// Gather statistics on post-transform cache misses for geometry
class OSGWTOOLS_EXPORT VertexCacheMissVisitor : public osg::NodeVisitor
{
public:
    VertexCacheMissVisitor(unsigned cacheSize = 16);
    void reset();
    virtual void apply(osg::Geode& geode);
    void doGeometry(osg::Geometry& geom);
    unsigned misses;
    unsigned triangles;
protected:
    const unsigned _cacheSize;
};

// Optimize the use of the GPU pre-transform cache by arranging vertex
// attributes in the order they are used.
class OSGWTOOLS_EXPORT VertexAccessOrderVisitor : public GeometryCollector
{
public:
    VertexAccessOrderVisitor(Optimizer* optimizer = 0)
        : GeometryCollector(optimizer,
            Optimizer::ALL_OPTIMIZATIONS /*Optimizer::VERTEX_PRETRANSFORM*/ )
    {
    }
    void optimizeOrder();
    void optimizeOrder(osg::Geometry& geom);
};

/**\}*/

}


// __OSGWTOOLS_MESH_OPTIMIZERS_H__
#endif
