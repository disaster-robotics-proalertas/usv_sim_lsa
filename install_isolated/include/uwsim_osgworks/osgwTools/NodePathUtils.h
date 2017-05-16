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

#ifndef __OSGWTOOLS_NODE_PATH_UTILS__
#define __OSGWTOOLS_NODE_PATH_UTILS__ 1


#include "osgwTools/Export.h"
#include <osg/Node>

#include <string>
#include <vector>


namespace osgwTools
{


/** \defgroup NodePathUtils Utilities for NodePaths

The set of utilities in \c NodePathUtils.h provides a way to identify
a unique Node in a scene graph without using Node addresses.

The osg::NodePath class identifies nodes in a scene graph using
a std::vector< osg::Node* > or a list of Node addresses. Because
node addresses change from one invocation of your application to the
next, NodePath is unsuitable for saving and restoring a node
identifier.

These utilities provide two alternative representations for a NodePath:
an IndexedNodePath and a std::string representation of an
IndexedNodePath.

The IndexedNodePath creates a vector of NodeData structs, using one
NodeData for every Node in the NodePath after the root node. NodeData
contains the Node child index, class name, and osg::Object name.
You use the functions nodePathToIndexed() and indexedToNodePath() to convert
between an osg::NodePath and an IndexedNodePath.

The functions nodePathToString() and stringToNodePath() provide a way to
serialize an IndexedNodePath to a string and vice versa.

Most applications do not use the IndexedNodePath, but instead 
convert directly between osg::NodePaths and std::strings, using the
functions nodePathToString() and stringToNodePath().

Two overloaded findNode() functions identify a specific node with 
either an IndexedNodePath or a string representation and a root node
to start searching from.

When the code attempts to find a node from an IndexedNodePath or string
representation, it uses the information in NodeData to find the
best possible match. In order of preference, the node returns as:
 -# A match for all three child index, class name, and object name
 -# A match for both class name and object name
 -# A match for child index only
 -# A match for either class name or object name
 
NULL returns if all data fails to match.

\test testnodepathutils
*/
/*@{*/


/** \brief Stores node data and is a basic element of the IndexedNodePath container.

\see NodePathUtils
*/
struct NodeData
{
    NodeData();
    NodeData( unsigned int index, const osg::Node& node );

    /** Searches the parent Group children for the best matching child node.
    Possible future work: Allow applications to specify a callback
    to override how to perform child identification.
    */
    osg::Node* findNode( osg::Group* parent ) const;

    bool operator==( const NodeData& rhs );
    bool operator!=( const NodeData& rhs );

    unsigned int _index;
    std::string _className;
    std::string _objectName;
};

/** \brief Container of NodeData structs.
This is a non-address based representation of an osg::NodePath.
Note that this representation is based on child indices.
It is always one element shorter than the equivalent NodePath,
because the NodePath root Node doesn't have a child index.
*/
typedef std::vector< NodeData > IndexedNodePath;


/** Converts from an osg::NodePath to an IndexedNodePath.
*/
OSGWTOOLS_EXPORT IndexedNodePath nodePathToIndexed( const osg::NodePath& nodePath );

/** Converts from an IndexedNodePath to an osg::NodePath.
\param indexedNodePath Input IndexedNodePath to convert.
\param root The returned NodePath stores this first and 
uses it as the parent for identifying the first Node in the input IndexedNodePath.
*/
OSGWTOOLS_EXPORT osg::NodePath indexedToNodePath( const IndexedNodePath& indexedNodePath, osg::Group* root );


/** Serializes an IndexedNodePath to a string.
*/
OSGWTOOLS_EXPORT std::string indexedToString( const IndexedNodePath& indexedNodePath );

/** Deserializes a string to an IndexedNodePath.
*/
OSGWTOOLS_EXPORT IndexedNodePath stringToIndexed( const std::string& stringPath );


/** Serializes an osg::NodePath to a string.
*/
OSGWTOOLS_EXPORT std::string nodePathToString( const osg::NodePath& nodePath );

/** Deserializes a string to an osg::NodePath.
\param stringPath Input string to deserialize.
\param root The returned NodePath stores this first and uses it as the parent for
identifying the first Node in the input string.
*/
OSGWTOOLS_EXPORT osg::NodePath stringToNodePath( const std::string& stringPath, osg::Group* root );


/** Identifies a Node from an IndexedNodePath and root Node.
*/
OSGWTOOLS_EXPORT osg::Node* findNode( const IndexedNodePath& indexedNodePath, osg::Group* root );

/** Identifies a Node from a string and root Node.
*/
OSGWTOOLS_EXPORT osg::Node* findNode( const std::string& stringPath, osg::Group* root );


#ifdef OSGWORKS_BUILD_TESTS

/** Combined black and white box testing routine. Exercises internal
utilities, such as the QuotedString class, stream IO for QuotedStrings
and NodeData structs, the findNode() functions, and conversion between
osg::NodePaths, IndexedNodePaths, and strings.

The testNodePathUtils test calls this function and is part of
the automated CTest regression test suite.

\return 0 for success. 1 for failure.
*/
OSGWTOOLS_EXPORT int testNodePathUtils();

// OSGWORKS_BUILD_TESTS
#endif


/*@}*/


// namespace osgwTools
}

// __OSGWTOOLS_NODE_PATH_UTILS__
#endif
