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

#include "osgwTools/RemoveLOD.h"
#include <osg/NodeVisitor>
#include <osg/LOD>
#include <osg/Version>

#include <osg/Notify>

#include <limits>


namespace osgwTools
{


RemoveLOD::RemoveLOD( const osg::NodeVisitor::TraversalMode travMode )
  : osg::NodeVisitor( travMode )
{
    osg::notify( osg::INFO ) << "RemoveLOD: This class is deprecated." << std::endl;
    osg::notify( osg::INFO ) << "  Please use CollapseLOD instead." << std::endl;
}
RemoveLOD::~RemoveLOD()
{
}



// examine all children, remove all but the highest-LOD one, and set its LOD range
// to be wide open. Finally set Rangemode to DISTANCE_FROM_EYE_POINT
void
RemoveLOD::apply( osg::LOD& node )
{
	// find highest LOD by examining each child, preserve copy of highest LOD in
	// temporary highestLOD ref_ptr while removing all the others

	osg::ref_ptr<osg::Node> highestLOD;
	float closestLODnear;

	if(node.getRangeMode() == osg::LOD::DISTANCE_FROM_EYE_POINT)
	{
		closestLODnear = std::numeric_limits<float>::max();
	} // if
	else
	{
		closestLODnear = std::numeric_limits<float>::min();
	} // else

	osg::notify( osg::DEBUG_INFO ) << "RemoveLOD LOD name: " << node.getName() << std::endl;
	osg::notify( osg::DEBUG_INFO ) << "RemoveLOD LOD NumChildren: " << node.getNumChildren() << std::endl;
	
	for(size_t childLoop = 0; childLoop < node.getNumChildren(); childLoop++)
	{

		osg::notify( osg::DEBUG_INFO ) << "  RemoveLOD child name: " << node.getChild(childLoop)->getName() << std::endl;
		osg::notify( osg::DEBUG_INFO ) << "  RemoveLOD child LODmin: " << node.getMinRange(childLoop) << std::endl;
		osg::notify( osg::DEBUG_INFO ) << "  RemoveLOD child LODmax: " << node.getMaxRange(childLoop) << std::endl;

		// does this child satisfy whichever "nearest" criteria in effect on this LOD?
		bool thisIsClosest = false;
		if(node.getRangeMode() == osg::LOD::DISTANCE_FROM_EYE_POINT)
		{
			// EYE_POINT means lower numbers are nearer
			if(node.getMinRange(childLoop) < closestLODnear) // here we're comparing MAX range
			{
				thisIsClosest = true;
			} // if
		} // if
		else
		{
			// PIXEL_SIZE_ON_SCREEN means higher numbers are nearer
			if(node.getMaxRange(childLoop) > closestLODnear) // now we're comparing MAX range
			{
				thisIsClosest = true;
			} // if
		} // else
		if(thisIsClosest)
		{
			osg::notify( osg::DEBUG_INFO ) << "   RemoveLOD closest: " << node.getChild(childLoop)->getName() << std::endl;
			// grab a reference to it so it survives the wholesale obliteration below
			highestLOD = node.getChild(childLoop); // increases ref count temporarily
			closestLODnear = node.getMinRange(childLoop);
		} // if
	} // for

	// wholesale obliterate all childern
	node.removeChildren(0, node.getNumChildren());

	// re-insert the highest-LOD with wide-open DISTANCE_FROM_EYE_POINT LOD range
	if(highestLOD.valid())
	{
		osg::notify( osg::DEBUG_INFO ) << "    RemoveLOD keeping child: " << highestLOD->getName() << std::endl;
		node.addChild(highestLOD.get(), 0.0, std::numeric_limits<float>::max());
	} // if

	// set to simpler, EYE_POINT mode (doesn't require calculation of onscreen pixel size)
	node.setRangeMode(osg::LOD::DISTANCE_FROM_EYE_POINT);
	
	// now move on to our children (only one at this point) and process them in case
	// they have LODs that need RemoveLOD
	traverse( node );
}



// osgwTools
}
