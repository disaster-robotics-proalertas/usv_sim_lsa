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

#ifndef __OSGWTOOLS_NODE_UTILS__
#define __OSGWTOOLS_NODE_UTILS__ 1


#include "osgwTools/Export.h"
#include <osg/Node>
#include <osg/Group>


namespace osgwTools
{


/** \defgroup NodeUtils Utilities for operating on Nodes and Groups
*/
/*@{*/

/** Copies the contents of a Group's children container to the other.
The actual child Nodes aren't duplicated, only the references to them.
This means the child nodes are multi-parented.

This does NOT delete existing children from the source node. You need to 
do this.

\return Count of children references copied. */
OSGWTOOLS_EXPORT unsigned int copyChildReferences( osg::Group* dest, osg::Group* source );

/** Replaces all ocurrences of the existingGraph node (which may or may not be
a Group with children) in the scene graph with the newGraph node (which also may
or may not have children). */
OSGWTOOLS_EXPORT void replaceSubgraph( osg::Node* newGraph, osg::Node* existingGraph );

/*@}*/


// namespace osgwTools
}

// __OSGWTOOLS_NODE_UTILS__
#endif
