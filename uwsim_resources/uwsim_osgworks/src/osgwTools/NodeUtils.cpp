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

#include "osgwTools/NodeUtils.h"
#include <osg/Node>
#include <osg/Group>


namespace osgwTools
{

unsigned int
copyChildReferences( osg::Group* dest, osg::Group* source )
{
	unsigned int nodeLoop = 0;
	for(; nodeLoop < source->getNumChildren(); nodeLoop++)
	{
		dest->addChild(source->getChild(nodeLoop));
	} // for
	return(nodeLoop);
} // copyChildReferences

void replaceSubgraph(osg::Node *newGraph, osg::Node *existingGraph)
{
	osg::Node::ParentList parentsCopy;
	parentsCopy = existingGraph->getParents(); // make a copy that won't be disturbed during our modifications below
	// add the newGraph to all parents of the existingGraph, and remove existingGraph from them
	for(unsigned int parentLoop = 0; parentLoop < parentsCopy.size(); parentLoop++)
	{
		osg::Group *existingParent = parentsCopy[parentLoop];
		if(existingParent)
		{
			existingParent->addChild(newGraph);
			existingParent->removeChild(existingGraph);
		} // if
	} // for
} // replaceSubgraph


// namespace osgwTools
}
