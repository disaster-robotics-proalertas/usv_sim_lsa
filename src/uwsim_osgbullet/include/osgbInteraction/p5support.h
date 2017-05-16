/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgBullet is (C) Copyright 2009-2011 by Kenneth Mark Bryden
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

#ifndef __OSGBINTERACTION_P5_SUPPORT_H__
#define __OSGBINTERACTION_P5_SUPPORT_H__ 1


#include <osgbInteraction/Export.h>
#include <osgbInteraction/HandNode.h>

#include "p5dll.h"


namespace osgbInteraction
{


/** \class P5Glove p5support.h <osgbInteraction/p5support.h>
\brief Support for the P5 data glove.

TBD. Should this derive from osg::Referenced?

*/
class OSGBINTERACTION_EXPORT P5Glove
{
public:
	P5Glove(bool rotation = false, bool filtering = true, int gloveIndex = 0);
	~P5Glove();

	bool updateHandState(osgbInteraction::HandNode* hn);
	void setFilteringEnable(bool newFilteringState) {_enableFiltering = newFilteringState;}
	void setRotationEnable(bool newRotationState) {_enableRotation = newRotationState;}
	bool getReady(void) const {return(_ready);}
	bool getVisible(void) const {return(_ready && (_state->Visible != 0));}
	void setMovementScale(const osg::Vec3 newScale) {_movementScale = newScale;}

private:
	static bool _initialized;
	static unsigned int _instances;
	bool _ready, _enableFiltering, _enableRotation;
	unsigned int _previousFrame;
	P5State* _state;
	osg::Vec3 _movementScale;
}; // P5Glove


// osgbInteraction
}


// __OSGBINTERACTION_P5_SUPPORT_H__
#endif
