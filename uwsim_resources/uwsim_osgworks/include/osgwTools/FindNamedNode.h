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

#ifndef __OSGWTOOLS_FIND_NAMED_NODE__
#define __OSGWTOOLS_FIND_NAMED_NODE__ 1


#include "osgwTools/Export.h"
#include <osg/NodeVisitor>
#include <string>


namespace osgwTools
{


/**
\brief Finds all Nodes with the given name.

Invokes like any standard osg::NodeVisitor:
subgraph->accept( findNamedNode );
or:
findNamedNode::apply( *subgraph );

This NodeVisitor searches for a matching name by examining the 
value of each osg::Node's osg::Object::getName() string. Whether a Node matches 
depends on the specified match algorithm. \see setMatchMethod.

Calling code accesses the matched Nodes and their paths using the
public _napl member variable. \see _napl.
*/
class OSGWTOOLS_EXPORT FindNamedNode : public osg::NodeVisitor
{
public:
    /**
    @param name Name of the Node to search for.
    */
    FindNamedNode( const std::string& name, const osg::NodeVisitor::TraversalMode travMode=osg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN );
    ~FindNamedNode();

    typedef std::pair< osg::Node*, osg::NodePath > NodeAndPath;
    typedef std::vector< NodeAndPath > NodeAndPathList;
    NodeAndPathList _napl;

    void reset();

    /**
    Algorithm for matching the specified name. Possible future
    work: support for case-insensitive matching.
    */
    typedef enum {
        EXACT_MATCH,
        CONTAINS
    } MatchMethod;
    /**
    Specifies the match algorithm.
    @param method The match algorithm. The default is EXACT_MATCH
    */
    void setMatchMethod( MatchMethod method );
    /**
    Gets the match algorithm.
    */
    MatchMethod getMatchMethod() const;

    /**
    Controls whether the named Node is included at the end of
    the NodePaths in _napl.
    @param includeTargetNode If false, don't include the named Node
    in the returned NodePaths. The default is true (include the named Node
    in the paths).
    */
    void setPathsIncludeTargetNode( bool includeTargetNode );
    /**
    Gets the current setting for including the named Node in the
    returned NodePaths.
    */
    bool getPathsIncludeTargetNode() const;

    /**
    Overrides of base class apply() method.
    */
    void apply( osg::Node& node );

protected:
    std::string _name;

    MatchMethod _method;
    bool _includeTargetNode;
};

// namespace osgwTools
}

// __OSGWTOOLS_FIND_NAMED_NODE__
#endif
