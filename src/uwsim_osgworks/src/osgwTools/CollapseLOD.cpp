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

#include "osgwTools/CollapseLOD.h"
#include "osgwTools/NodeUtils.h"
#include <osg/NodeVisitor>
#include <osg/LOD>
#include <osg/Version>

#include <limits>
#include <cassert>


namespace osgwTools
{


CollapseLOD::CollapseLOD(NodeSelectorCallback *SelectorCallback, const CollapseMode collapseMode,  const osg::NodeVisitor::TraversalMode travMode )
  : osg::NodeVisitor( travMode ),
    _LODsLocated(0),
    _LODsProcessed(0),
    _depth(0),
    _collapseMode(collapseMode),
    _selectorCallback(SelectorCallback)
{
    if( _selectorCallback==NULL )
        _selectorCallback = new HighestLODChildSelectorCallback;
}

unsigned int CollapseLOD::finishProcessingLODs(void)
{
    // process all the stored references to LOD nodes
	for(NodeSet::iterator itr=_collectedLODNodes.begin();
        itr!=_collectedLODNodes.end();
        ++itr)
    {
		osg::notify( osg::DEBUG_INFO ) << "CollapseLOD::finishProcessingLODs LOD name: " << (*itr)->getName() << std::endl;

		osg::ref_ptr<osg::Group> currentNodeAsGroup = (*itr)->asGroup();
		if(currentNodeAsGroup.valid())
		{
			osg::notify( osg::DEBUG_INFO ) << "CollapseLOD::finishProcessingLODs LOD NumChildren: " << currentNodeAsGroup->getNumChildren() << std::endl;
			// invoke callback to determine which child should be retained
			osg::Node *selectedChild = _selectorCallback->selectChild(currentNodeAsGroup.get());
			if(_collapseMode == COLLAPSE_TO_GROUP) // <<<>>> what if parent of parent is an LOD itself?
			{
				osg::ref_ptr<osg::Group> newGroup = new osg::Group(*(currentNodeAsGroup.get()));
				if(newGroup.valid())
				{
					// ensure only the desired child is retained in the new Group
					newGroup->removeChildren(0, newGroup->getNumChildren()); // would be nice if we could have prevent them from being copied during the constructor
					if(selectedChild)
					{
						newGroup->addChild(selectedChild);
					} // if
					// replace existing LOD node with replacement Group node
					osgwTools::replaceSubgraph(newGroup.get(), (*itr).get());
				} // if
			} // if
			else
			{ // currently must be COLLAPSE_TO_PARENT
				// get a scratch copy we can continue to rely on while adding/removing with the live copy
				osg::Node::ParentList unchangedParents = currentNodeAsGroup->getParents();
				// add selectedChild to all Parents of current node and remove current node
				// if current node HAS no parents (it's the root of a graph) this silently does nothing
				for(osg::Node::ParentList::iterator parentIter = unchangedParents.begin();
					parentIter != unchangedParents.end(); parentIter++)
				{
					// is the parent node itself an LOD?
					osg::Node *parentTestType = *parentIter;
					osg::LOD *parentAsLOD = dynamic_cast<osg::LOD *>(parentTestType);
					if(parentAsLOD)
					{
						// find the range values for the child node (an LOD) we're about to eliminate
						float originalChildMin, originalChildMax;

						// first find which child number the LOD-to-be-eliminated is, so we can query the ranges by ordinal
						// which is the only API OSG offers for accessing the ranges
						unsigned int childIndex = parentAsLOD->getChildIndex((*itr).get());
						originalChildMin = parentAsLOD->getMinRange(childIndex);
						originalChildMax = parentAsLOD->getMaxRange(childIndex);
						// add selected child node to parent of LOD we're eliminating, using LOD ranges of the LOD we're eliminating
						// According to OSG's LOD docs, "Children can be in any order, and don't need to be sorted by range or amount of detail."
						// so just sticking this onto the end should be fine.
						parentAsLOD->addChild(selectedChild, originalChildMin, originalChildMax);
						// remove original LOD node
						(*parentIter)->removeChild((*itr).get());
					} // if
					else
					{
						// add selected child node
						(*parentIter)->addChild(selectedChild);
						// remove previous LOD node
						(*parentIter)->removeChild(currentNodeAsGroup.get());
					} // else
				} // for
			} // else
			_LODsProcessed++;
		} // if
    } // for

	return(_LODsProcessed);
} // CollapseLOD::finishProcessingLODs


void CollapseLOD::processNode( osg::Node& node )
{
    _depth++;
    traverse( node );
    _depth--;

	if( _depth == 0 )
	{
		// <<<>>> This does not work because we're in the midst of a traversal, and it breaks the iterator.
		// This code is left in-place in case we determine a way to accomplish this later.
        //finishProcessingLODs();
	} // if
} // CollapseLOD::processNode

// Only here to do depth-tracking processing on all node types
void CollapseLOD::apply( osg::Node& node )
{
    processNode( node );
}


// add all LOD nodes to a collection container for later processing
void
CollapseLOD::apply( osg::LOD& node )
{

	osg::notify( osg::DEBUG_INFO ) << "CollapseLOD apply() LOD name: " << node.getName() << std::endl;
	osg::notify( osg::DEBUG_INFO ) << "CollapseLOD apply() LOD NumChildren: " << node.getNumChildren() << std::endl;

	_collectedLODNodes.insert(&node);
	_LODsLocated++;

    processNode( node );

} // CollapseLOD::apply(LOD)



// osgwTools
}
