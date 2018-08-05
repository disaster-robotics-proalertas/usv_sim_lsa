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

// Copyright (c) 2010 AlphaPixel LLC. All rights reserved.
// Inspired by code from TestDevP5.cpp
// This code relies on the driver in A__DualModeDriverBeta31rd.zip, which includes the necessary
// header file, .lib file and .DLL file which are not checked into osgBullet for license reasons.
// Much of this code is inspired from TestDevP5.cpp form the same distribution.


#include <osgbInteraction/p5support.h>
#include <osg/Notify>
#include <osg/Vec2f>
#include <osg/Quat>
#include <osgwTools/Quat.h>
#include <limits>


namespace osgbInteraction
{


bool P5Glove::_initialized = false;
unsigned int P5Glove::_instances = 0;

P5Glove::P5Glove(bool rotation, bool filtering, int gloveIndex)
: _state(NULL), _ready(false), _enableFiltering(filtering), _enableRotation(rotation)
{

	if(!_initialized)
	{
		// Initialise the glove(s)
		// This initialises and starts the driver, it is essential.
		P5_Init();
		_initialized = true;

		// get the number of gloves
		int count = P5_GetCount();
        osg::notify( osg::DEBUG_INFO ) << "Detected " << count << " P5 gloves connected." << std::endl;

		// disable mouse mode
		P5_SetMouseState(-1, false);
	} // if

	_instances++; // track number of P5Glove objects currently using P5Glove DLL interface

	// get the state data structure pointer for the specified glove
	// this data structure is updated automatically.
	if(_state = P5_GetStatePointer(gloveIndex))
	{
		// Set the units to centimetres
		P5_SetUnits(P5_CM);

		// In the previous BETA 1 version you had to call this
		// before you could use FilterPos. Now it is the default, so it isn't needed.
		// P5_SetFilterMode(P5_FILTER_AVERAGE | P5_FILTER_DEADBAND);

		// remember what the previous frame was, so we know when we've seen the next one
		_previousFrame = 0;

		_ready = true;
	} // if

} // P5Glove::P5Glove

P5Glove::~P5Glove()
{
	_instances--; // decrement instance counter

	if(_initialized && !_instances)
	{
		// Close the glove(s)
		P5_Close();
		// restore mouse state
		P5_RestoreMouse(-1);
		_initialized = false;
	} // if

} // P5Glove::~P5Glove

// returns false to indicate error abort
bool P5Glove::updateHandState( osgbInteraction::HandNode* hn )
{
	if(!_ready) return(false);

	if(_state->Frame == _previousFrame) return(true); // no new data available, silently exit
	_previousFrame = _state->Frame; // update frame counter

	osgbInteraction::HandNode::AllParams currentGloveState;

	if(_enableFiltering)
	{
		currentGloveState._pos = osg::Vec3(_state->FilterPos[0], _state->FilterPos[2], _state->FilterPos[1]);

		/*
		// 3x3 rotation matrix to osg::Quat (this didn't work)
		currentGloveState._att.set(
				osg::Matrixf(
				_state->FilterRotMat[0][0], _state->FilterRotMat[0][1], _state->FilterRotMat[0][2], 0.0f,
				_state->FilterRotMat[1][0], _state->FilterRotMat[1][1], _state->FilterRotMat[1][2], 0.0f,
				_state->FilterRotMat[2][0], _state->FilterRotMat[2][1], _state->FilterRotMat[2][2], 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f
				)
			);
			*/
		if(_enableRotation)
		{
			currentGloveState._att = osgwTools::makeHPRQuat( _state->FilterYaw, -_state->FilterPitch, -_state->FilterRoll );
		} // if
		else
		{
			currentGloveState._att = osgwTools::makeHPRQuat( 180.0, 0.0, 0.0 );
		} // else

	} // if
	else
	{
		currentGloveState._pos = osg::Vec3(_state->x, _state->z, _state->y);

		/*
		// 3x3 rotation matrix to osg::Quat (this didn't work)
		currentGloveState._att.set(
				osg::Matrixf(
				_state->RotMat[0][0], _state->RotMat[0][1], _state->RotMat[0][2], 0.0f,
				_state->RotMat[1][0], _state->RotMat[1][1], _state->RotMat[1][2], 0.0f,
				_state->RotMat[2][0], _state->RotMat[2][1], _state->RotMat[2][2], 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f
				)
			);
			*/
		if(_enableRotation)
		{
			currentGloveState._att = osgwTools::makeHPRQuat( _state->yaw, -_state->pitch, -_state->roll );
		} // if
		else
		{
			currentGloveState._att = osgwTools::makeHPRQuat( 180.0, 0.0, 0.0 );
		} // else
	} // else

	// apply movement scaling
    currentGloveState._pos = osg::Vec3( _movementScale.x() * currentGloveState._pos.x(),
        _movementScale.y() * currentGloveState._pos.y(),
        _movementScale.z() * currentGloveState._pos.z() );

	// update finger curl
	// we only have one composite curl, so we spread it equally between the
	// _ROTATE_MIDDLE and _ROTATE_OUTER components of the osg::Vec2f for each
	// finger in AllParams

	float fingerRange;
	fingerRange = ((float)_state->finger[0] / 64.0f); // scale [0 ... 64] into [0 ... 1]
	currentGloveState._finger0 = osg::Vec2f(fingerRange, fingerRange); // thumb
	fingerRange = ((float)_state->finger[1] / 64.0f); // scale [0 ... 64] into [0 ... 1]
	currentGloveState._finger1 = osg::Vec2f(fingerRange, fingerRange); // pointer
	fingerRange = ((float)_state->finger[2] / 64.0f); // scale [0 ... 64] into [0 ... 1]
	currentGloveState._finger2 = osg::Vec2f(fingerRange, fingerRange); // middle
	fingerRange = ((float)_state->finger[3] / 64.0f); // scale [0 ... 64] into [0 ... 1]
	currentGloveState._finger3 = osg::Vec2f(fingerRange, fingerRange); // ring
	fingerRange = ((float)_state->finger[4] / 64.0f); // scale [0 ... 64] into [0 ... 1]
	currentGloveState._finger4 = osg::Vec2f(fingerRange, fingerRange); // pinky

	// <<<>>> update node visibility/endabled state based on glove visibility?

	// set current glove state to be HandNode state
	hn->setAll(currentGloveState);

	return(true);
} // P5Glove::updateHandState


// osgbInteraction
}
