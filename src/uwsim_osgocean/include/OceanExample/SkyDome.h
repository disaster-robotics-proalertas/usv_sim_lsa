/*
* This source file is part of the osgOcean library
* 
* Copyright (C) 2009 Kim Bale
* Copyright (C) 2009 The University of Hull, UK
* 
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU Lesser General Public License as published by the Free Software
* Foundation; either version 3 of the License, or (at your option) any later
* version.

* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
* http://www.gnu.org/copyleft/lesser.txt.
*/

#pragma once
#include "SphereSegment.h"
#include <osg/Program>
#include <osg/Uniform>
#include <osg/TextureCubeMap>

class SkyDome : public SphereSegment
{
public:
	SkyDome( void );
	SkyDome( const SkyDome& copy, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
	SkyDome( float radius, unsigned int longSteps, unsigned int latSteps, osg::TextureCubeMap* cubemap );

protected:
	~SkyDome(void);
	
public:
	void setupStateSet( osg::TextureCubeMap* cubemap );
	void create( float radius, unsigned int latSteps, unsigned int longSteps, osg::TextureCubeMap* cubemap );
	
	inline void setCubeMap( osg::TextureCubeMap* cubemap ){
		getOrCreateStateSet()->setTextureAttributeAndModes( 0, cubemap, osg::StateAttribute::ON );
	}

private:
	osg::ref_ptr<osg::Program> createShader(void);

};
