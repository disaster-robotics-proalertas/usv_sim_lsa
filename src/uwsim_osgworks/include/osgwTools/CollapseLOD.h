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

#ifndef __OSGWTOOLS_COLLAPSE_LOD__
#define __OSGWTOOLS_COLLAPSE_LOD__ 1


#include "osgwTools/Export.h"
#include <osg/NodeVisitor>
#include <osg/Group> // for NodeList type
#include <osg/LOD> // for NodeList type
#include <osg/Notify>

#include <string>
#include <limits>

//This is necessary to resolve windows min/max macros compile errors 
//that are caused by header pollution through downstream inclusion.
#undef min
#undef max

namespace osg {
    class Geometry;
}


namespace osgwTools
{


// Should this perhaps be exposed somewhere else for wider use?
typedef std::set< osg::ref_ptr<osg::Node> > NodeSet;

/** NodeSelectorCallback NodeSelectorCallback.h <osgwTools/NodeSelectorCallback.h>
\brief Callback to decide which LOD child to retain */
class OSGWTOOLS_EXPORT NodeSelectorCallback : public osg::Referenced
{
    public:
        /** Returns the Node that should be selected for retention or NULL if no Nodes should be retained.
		Override to implement the desired behavior. */
		virtual osg::Node *selectChild(osg::Group *parentOfCandidates) const
        {
            return NULL;
        } // selectChild
    
        virtual ~NodeSelectorCallback() {}
}; // NodeSelectorCallback



/** \brief Callback to select highest LOD child */
class HighestLODChildSelectorCallback : public osgwTools::NodeSelectorCallback
{
    public:
        /** Returns the child with the highest LOD. */
		osg::Node *selectChild(osg::Group *parentOfCandidates) const
        {
			osg::Node *selectedChild = NULL;
			if(parentOfCandidates)
			{
				if(osg::LOD *parentAsLOD = dynamic_cast<osg::LOD*>(parentOfCandidates))
				{
					// choose highest LOD
					float closestLODnear;
					if(parentAsLOD->getRangeMode() == osg::LOD::DISTANCE_FROM_EYE_POINT)
					{
						closestLODnear = std::numeric_limits<float>::max();
					} // if
					else
					{
						closestLODnear = std::numeric_limits<float>::min();
					} // else
					for(size_t childLoop = 0; childLoop < parentAsLOD->getNumChildren(); childLoop++)
					{
						osg::notify( osg::DEBUG_INFO ) << "  HighestLODChildSelectorCallback child name: " << parentAsLOD->getChild(childLoop)->getName() << std::endl;
						osg::notify( osg::DEBUG_INFO ) << "  HighestLODChildSelectorCallback child LODmin: " << parentAsLOD->getMinRange(childLoop) << std::endl;
						osg::notify( osg::DEBUG_INFO ) << "  HighestLODChildSelectorCallback child LODmax: " << parentAsLOD->getMaxRange(childLoop) << std::endl;

						// does this child satisfy whichever "nearest" criteria in effect on this LOD?
						bool thisIsClosest = false;
						if(parentAsLOD->getRangeMode() == osg::LOD::DISTANCE_FROM_EYE_POINT)
						{
							// EYE_POINT means lower numbers are nearer
							if(parentAsLOD->getMinRange(childLoop) < closestLODnear) // here we're comparing MAX range
							{
								thisIsClosest = true;
							} // if
						} // if
						else
						{
							// PIXEL_SIZE_ON_SCREEN means higher numbers are nearer
							if(parentAsLOD->getMaxRange(childLoop) > closestLODnear) // now we're comparing MAX range
							{
								thisIsClosest = true;
							} // if
						} // else
						if(thisIsClosest)
						{
							osg::notify( osg::DEBUG_INFO ) << "   HighestLODChildSelectorCallback closest: " << parentAsLOD->getChild(childLoop)->getName() << std::endl;
							// grab a reference to it so it survives the wholesale obliteration below
							selectedChild = parentAsLOD->getChild(childLoop);
							closestLODnear = parentAsLOD->getMinRange(childLoop);
						} // if
					} // for
				} // if
			} // if
            return selectedChild;
        } // selectChild
}; // HighestLODChildSelectorCallback


/** \class CollapseLOD CollapseLOD.h <osgwTools/CollapseLOD.h>
\brief Removes LOD Nodes from a scene graph.

In typical usage, the application should invoke this visitor as with any OSG NodeVisitor, and
after traversal returns, the application should call the \ref finishProcessingLODs method.

This NodeVisitor has two modes of operation. It can replace LOD Nodes with Group Nodes, or it
can simply remove the LOD Nodes altogether. In either case, only the highest LOD child as
determined by the NodeSelectorCallback, remains (either attached to the new Group or attached
to the removed LOD's former parent). 

See \ref CollapseMode to control the mode of operation. COLLAPSE_TO_GROUP is useful when the
LOD Node contains callbacks, UserData, DescriptionLists, etc. This information is
copied from the LOD to the new Group. COLLAPSE_TO_PARENT (the default) removes the LOD and
the references to any such associated data.

\test testvisitors

*/
class OSGWTOOLS_EXPORT CollapseLOD : public osg::NodeVisitor
{
public:
	/** CollapseMode documentation TBD. */

    enum CollapseMode {
        COLLAPSE_TO_GROUP,
        COLLAPSE_TO_PARENT,
        NUM_COLLAPSE_MODES
    };

    /** Constructor
    \param SelectorCallback Callback to determine which child is the highest LOD. By default, 
	the function uses an instance of \ref HighestLODChildSelectorCallback.
    \param CollapseMode LOD removal method. Default is COLLAPSE_TO_PARENT. The function 
	removes the LODs and attaches the highest LOD child to the the LOD former parent(s).
    \param TraversalMode The traversal mode. The default is to traverse all children.
    */
	CollapseLOD( NodeSelectorCallback *SelectorCallback=NULL,
        const CollapseMode collapseMode=COLLAPSE_TO_PARENT,
        const osg::NodeVisitor::TraversalMode travMode=osg::NodeVisitor::TRAVERSE_ALL_CHILDREN );
	virtual ~CollapseLOD() {};

	void apply( osg::LOD& node );
	void apply( osg::Node& node );

	unsigned int getLODsLocated(void) const {return(_LODsLocated);}
	unsigned int getLODsProcessed(void) const {return(_LODsProcessed);}

    /** Calling code must call this function to remove LODs.
    This NodeVisitor collects LOD Nodes, but doesn't actually process/remove
    them until the calling code executes this member function. */
	unsigned int finishProcessingLODs(void);

protected:
	void processNode( osg::Node& node );

	NodeSet _collectedLODNodes;
	unsigned int _LODsLocated, _LODsProcessed, _depth;
	CollapseMode _collapseMode;
	osg::ref_ptr<NodeSelectorCallback> _selectorCallback;

};


// osgwTools
}

// __OSGWTOOLS_COLLAPSE_LOD__
#endif
