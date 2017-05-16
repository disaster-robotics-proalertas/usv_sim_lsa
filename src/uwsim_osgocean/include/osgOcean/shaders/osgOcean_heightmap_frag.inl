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

static const char osgOcean_heightmap_frag[] =
	"// osgOcean uniforms\n"
	"// -------------------\n"
	"uniform float osgOcean_WaterHeight;\n"
	"// -------------------\n"
	"\n"
	"varying vec4 vWorldVertex;\n"
	"\n"
	"void main(void)\n"
	"{\n"
	"	// Store the water depth\n"
	"	// maximum possible depth is 500,\n"
	"	// (a higher water depth value would not have a visible effect anyway)\n"
	"	gl_FragDepth = clamp((osgOcean_WaterHeight - vWorldVertex.z) / 500.0, 0.0, 1.0);\n"
	"\n"
	"	return;\n"
	"}\n";
