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

// ------------------------------------------------------------------------------
// -- THIS FILE HAS BEEN CREATED AS PART OF THE BUILD PROCESS -- DO NOT MODIFY --
// ------------------------------------------------------------------------------

static const char osgOcean_gaussian2_frag[] =
	"#extension GL_ARB_texture_rectangle : enable\n"
	"\n"
	"uniform sampler2DRect osgOcean_GaussianTexture;\n"
	"\n"
	"void main( void )\n"
	"{\n"
	"   vec2 texCoordSample = vec2( 0.0 );\n"
	"\n"
	"   vec4 color = 0.5 * texture2DRect(osgOcean_GaussianTexture, gl_TexCoord[0] );\n"
	"\n"
	"   texCoordSample.y = gl_TexCoord[0].y;\n"
	"   texCoordSample.x = gl_TexCoord[0].x + 1;\n"
	"   color += 0.25 * texture2DRect(osgOcean_GaussianTexture, texCoordSample);\n"
	"\n"
	"   texCoordSample.x = gl_TexCoord[0].x - 1;\n"
	"   color += 0.25 * texture2DRect(osgOcean_GaussianTexture, texCoordSample);\n"
	"\n"
	"   gl_FragColor = color;\n"
	"}\n";
