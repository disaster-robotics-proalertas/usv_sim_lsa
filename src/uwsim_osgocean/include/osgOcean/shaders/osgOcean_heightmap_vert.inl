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

static const char osgOcean_heightmap_vert[] =
	"// osgOcean uniforms\n"
	"// -------------------\n"
	"uniform float osgOcean_WaterHeight;\n"
	"// ------------------\n"
	"\n"
	"uniform mat4 osg_ViewMatrixInverse;\n"
	"uniform mat4 osg_ViewMatrix;\n"
	"\n"
	"varying vec4 vWorldVertex;\n"
	"\n"
	"void main(void)\n"
	"{\n"
	"	// Transform the vertex into world space\n"
	"	vWorldVertex = (osg_ViewMatrixInverse * gl_ModelViewMatrix) * gl_Vertex;\n"
	"	vWorldVertex.xyzw /= vWorldVertex.w;\n"
	"\n"
	"	// Project the vertex onto the ocean plane\n"
	"	vec4 projectedVertex = vWorldVertex;\n"
	"	projectedVertex.z = osgOcean_WaterHeight;\n"
	"\n"
	"	gl_Position = (gl_ProjectionMatrix * osg_ViewMatrix) * projectedVertex;\n"
	"\n"
	"	return;\n"
	"}\n";
