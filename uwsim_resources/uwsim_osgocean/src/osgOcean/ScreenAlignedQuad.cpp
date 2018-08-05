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

#include <osgOcean/ScreenAlignedQuad>

using namespace osgOcean;

ScreenAlignedQuad::ScreenAlignedQuad(void)
{
}

ScreenAlignedQuad::ScreenAlignedQuad( const osg::Vec3f& corner, const osg::Vec2f& dims, const osg::Vec2s& textureSize )
{
	build( corner, dims, textureSize );
}

ScreenAlignedQuad::ScreenAlignedQuad( const osg::Vec3f& corner, const osg::Vec2f& dims, osg::TextureRectangle* texture )
{
	if(texture)
		build( corner, dims, texture );
}

ScreenAlignedQuad::ScreenAlignedQuad( const ScreenAlignedQuad &copy, const osg::CopyOp &copyop):
	osg::Geometry(copy,copyop)
{
}

void ScreenAlignedQuad::build( const osg::Vec3f& corner, const osg::Vec2f& dims, const osg::Vec2s& textureSize )
{
	osg::Vec3Array* coords = new osg::Vec3Array(4);
	(*coords)[0] = corner+osg::Vec3f(0.f, dims.y(), 0.f);
	(*coords)[1] = corner;
	(*coords)[2] = corner+osg::Vec3f(dims.x(), 0.f, 0.f);
	(*coords)[3] = corner+osg::Vec3f(dims.x(), dims.y(), 0.f);
	setVertexArray(coords);

	osg::Vec2Array* tcoords = new osg::Vec2Array(4);
	(*tcoords)[0].set(0.f,             textureSize.y());
	(*tcoords)[1].set(0.f,             0.f);
	(*tcoords)[2].set(textureSize.x(), 0.f);
	(*tcoords)[3].set(textureSize.x(), textureSize.y());
	setTexCoordArray(0,tcoords);

	osg::Vec4Array* colours = new osg::Vec4Array(1);
	(*colours)[0].set(1.0f,1.0f,1.0,1.0f);
	setColorArray(colours);
	setColorBinding(osg::Geometry::BIND_OVERALL);

	osg::Vec3Array* normals = new osg::Vec3Array(1);
	(*normals)[0] = osg::Vec3f(0.f, -1.f, 0.f);
	(*normals)[0].normalize();
	setNormalArray(normals);
	setNormalBinding(osg::Geometry::BIND_OVERALL);

	addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));

	getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF );
}

void ScreenAlignedQuad::build( const osg::Vec3f& corner, const osg::Vec2f& dims, osg::TextureRectangle* texture )
{
	if(texture)
	{
		osg::Vec2s texSize( texture->getTextureWidth(), texture->getTextureHeight() );
		build(corner, dims, texSize);
	}
}
