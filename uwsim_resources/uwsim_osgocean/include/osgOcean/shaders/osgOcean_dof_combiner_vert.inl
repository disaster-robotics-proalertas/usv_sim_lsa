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

static const char osgOcean_dof_combiner_vert[] =
	"uniform vec2 osgOcean_ScreenRes;\n"
	"uniform vec2 osgOcean_LowRes;\n"
	"\n"
	"void main( void )\n"
	"{\n"
	"	gl_TexCoord[0] = gl_MultiTexCoord0 * vec4( osgOcean_ScreenRes, 1.0, 1.0 );\n"
	"	gl_TexCoord[1] = gl_MultiTexCoord0 * vec4( osgOcean_LowRes,    1.0, 1.0 );\n"
	"\n"
	"	gl_Position = ftransform();\n"
	"}\n"
	"\n";
