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

static const char osgOcean_godray_screen_blend_vert[] =
	"varying vec3 vRay;\n"
	"\n"
	"void main( void )\n"
	"{\n"
	"	gl_Position = gl_Vertex;\n"
	"\n"
	"	gl_TexCoord[0] = gl_MultiTexCoord0;\n"
	"\n"
	"	gl_TexCoord[1].xy = gl_TexCoord[0].st;\n"
	"	gl_TexCoord[1].zw = gl_TexCoord[0].st + vec2(1.0, 0.0);\n"
	"	gl_TexCoord[2].xy = gl_TexCoord[0].st + vec2(1.0, 1.0);\n"
	"	gl_TexCoord[2].zw = gl_TexCoord[0].st + vec2(0.0, 1.0);\n"
	"\n"
	"	vRay = gl_Normal;\n"
	"}\n";
