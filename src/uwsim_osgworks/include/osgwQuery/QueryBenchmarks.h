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

#ifndef __OSGWQUERY_QUERY_BENCHMARKS_H__
#define __OSGWQUERY_QUERY_BENCHMARKS_H__ 1


#include <osgwQuery/Export.h>
#include <osg/Referenced>
#include <osg/Camera>

namespace osg {
    class RenderInfo;
    class Drawable;
}


namespace osgwQuery
{


class QueryAPI;


/** \addtogroup GutheQuery */
/*@{*/


/** \class QueryBenchmarks QueryBenchmarks.h <osgwQuery/QueryBenchmarks.h>
\brief OpenGL occlusion query benchmarks used in Guthe algorithm.
*/
class OSGWQUERY_EXPORT QueryBenchmarks : public osg::Referenced
{
public:
    QueryBenchmarks( unsigned int contextID, osg::RenderInfo* ri );

    // Benchmark values. See Guthe, Section 3.2, Table 2.
    double _trSetup, _trTriangle, _trFragment;
    double _toSetup, _toFragment;
    double _toLatency, _toOverhead;

protected:
    ~QueryBenchmarks();

    double time( osg::Drawable* draw, osg::RenderInfo& ri );
    double time( osg::Drawable* draw, osg::RenderInfo& ri, osgwQuery::QueryAPI* qapi );

    void internalInit( unsigned int contextID, osg::RenderInfo* ri );
    bool _initialized;
};


/** Function to obtain a QueryBenchmarks class for a specific context.
*/
OSGWQUERY_EXPORT QueryBenchmarks* getQueryBenchmarks( unsigned int contextID, osg::RenderInfo* ri=NULL );


/** \class InitCallback QueryBenchmarks.h <osgwQuery/QueryBenchmarks.h>
\brief A Camera pre-draw callback to gather Guthe benchmark values.
*/
struct OSGWQUERY_EXPORT InitCallback : public osg::Camera::DrawCallback
{
    InitCallback() : _initialized( false ) {}

    virtual void operator()( osg::RenderInfo& renderInfo ) const;

    mutable bool _initialized;
};

/*@}*/


// osgwQuery
}

// __OSGWQUERY_QUERY_BENCHMARKS_H__
#endif
