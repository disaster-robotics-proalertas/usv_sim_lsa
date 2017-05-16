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

#ifndef __OSGWQUERY_QUERY_UTILS_H__
#define __OSGWQUERY_QUERY_UTILS_H__ 1


#include <osgwQuery/Export.h>
#include <osgwQuery/QueryComputation.h>
#include <osgwQuery/QueryObject.h>
#include <osgwQuery/QueryStats.h>
#include <osg/Drawable>
#include <OpenThreads/Mutex>

#include <osg/Node>
#include <osg/Geode>
#include <osg/NodeVisitor>
#include <osg/buffered_value>


namespace osgwQuery
{


/** \addtogroup GutheQuery */
/*@{*/


/** \brief osg::RenderBin name for front-to-back rendering.
Occlusion query benefits from front-to-back render order. This string is the
OSG RenderBin name for front-to-back rendering. Prior to OSG v3.0.0, osgwQuery
creates a static RenderBin prototype, but in v3.0.0 and later, OSG predefines
a front-to-back bin, and osgwQuery just uses that bin. The name is the same in
either case. */
#define _QUERY_FRONT_TO_BACK_BIN_NAME "SORT_FRONT_TO_BACK"


/** \class QueryCullCallback QueryUtils.h <osgwQuery/QueryUtils.h>
\brief Make a cull-time traverse/no-traverse decision based on Guthe
algorithm criteria in the QueryComputation class.
*/
class OSGWQUERY_EXPORT QueryCullCallback : public osg::NodeCallback
{
public:
    QueryCullCallback();
    QueryCullCallback( const QueryCullCallback& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgwQuery,QueryCullCallback);

    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv );

    void attach( osg::Node* node, osgwQuery::QueryComputation* nd );

    osgwQuery::QueryComputation* getQueryComputation() const { return( _nd ); }

protected:
    osg::Node* _node;
    osgwQuery::QueryComputation* _nd;
};
/** \class CameraResetCallback QueryUtils.h <osgwQuery/QueryUtils.h>
\brief Resets the Guthe screen coverage CscrOi value for each
Camera as a post-draw callback.
*/
struct OSGWQUERY_EXPORT CameraResetCallback : public osg::Camera::DrawCallback
{
public:
    CameraResetCallback();
    virtual void operator()( osg::RenderInfo& renderInfo ) const;
};


/** \class AddQueries QueryUtils.h <osgwQuery/QueryUtils.h>
\brief Add cull-time callbacks to a scene graph to support the Guthe algorithm.
*/
class OSGWQUERY_EXPORT AddQueries : public osg::NodeVisitor
{
public:
    AddQueries( osg::NodeVisitor::TraversalMode mode=osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
      : osg::NodeVisitor( mode ),
        _queryCount( 0 )
    {}

    virtual void apply( osg::Group& node );
    virtual void apply( osg::Geode& node );
    virtual void apply( osg::Camera& node );

    void setQueryStats( osgwQuery::QueryStats* qs ) { _qs = qs; }

    unsigned int getQueryCount() const { return( _queryCount ); }

protected:
    /** Provides optimized accumulation of per-node vertices and bounding box. The alternative
    is to run the CountsVisitor and ComputeBoundsVisitor at every Node, which would be
    innefficient and delay init time severely for large models. */
    void addDataToNodePath( osg::NodePath& np, unsigned int numVertices, const osg::BoundingBox& bb );

    osgwQuery::QueryStats* _qs;

    unsigned int _queryCount;
};

/** \class RemoveQueries QueryUtils.h <osgwQuery/QueryUtils.h>
\brief Remove the effects of an AddQueries visitor.
*/
class OSGWQUERY_EXPORT RemoveQueries : public osg::NodeVisitor
{
public:
    RemoveQueries( osg::NodeVisitor::TraversalMode mode=osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
      : osg::NodeVisitor( mode )
    {}

    virtual void apply( osg::Node& node );
    virtual void apply( osg::Camera& node );

protected:
};

/*@}*/


// osgwQuery
}

// __OSGWQUERY_QUERY_UTILS_H__
#endif
