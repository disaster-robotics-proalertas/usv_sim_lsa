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

static const char osgOcean_silt_points_vert[] =
	"uniform float osgOcean_InversePeriod;\n"
	"uniform vec4 osgOcean_ParticleColour;\n"
	"uniform float osgOcean_ParticleSize;\n"
	"\n"
	"uniform float osg_SimulationTime;\n"
	"\n"
	"varying vec4 colour;\n"
	"\n"
	"void main(void)\n"
	"{\n"
	"	float startTime = gl_MultiTexCoord1.x;\n"
	"\n"
	"	vec4 v_current = gl_Vertex;\n"
	"\n"
	"	float disp = (osg_SimulationTime - startTime)*osgOcean_InversePeriod;\n"
	"\n"
	"	vec3 direction = sign(gl_Normal);\n"
	"\n"
	"	v_current.x = direction.x * fract( disp + gl_Vertex.x );\n"
	"	v_current.y = direction.y * fract( disp + gl_Vertex.y );\n"
	"	v_current.z = direction.z * fract( disp + gl_Vertex.z );\n"
	"\n"
	"	colour = osgOcean_ParticleColour;\n"
	"\n"
	"	gl_Position = gl_ModelViewProjectionMatrix * v_current;\n"
	"\n"
	"	float pointSize = abs(1280.0*osgOcean_ParticleSize / gl_Position.w);\n"
	"\n"
	"	gl_PointSize = ceil(pointSize);\n"
	"\n"
	"	colour.a = 0.05+(pointSize*pointSize)/(gl_PointSize*gl_PointSize);\n"
	"\n"
	"	gl_ClipVertex = v_current;\n"
	"}\n";
