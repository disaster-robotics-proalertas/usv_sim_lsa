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

static const char osgOcean_silt_quads_vert[] =
	"uniform vec4 osgOcean_ParticleColour;\n"
	"\n"
	"uniform float osgOcean_InversePeriod;\n"
	"uniform float osgOcean_ParticleSize;\n"
	"uniform float osg_SimulationTime;\n"
	"uniform float osg_DeltaSimulationTime;\n"
	"\n"
	"varying vec4 colour;\n"
	"varying vec2 texCoord;\n"
	"\n"
	"void main(void)\n"
	"{\n"
	"    float startTime = gl_MultiTexCoord1.x;\n"
	"    texCoord = gl_MultiTexCoord0.xy;\n"
	"\n"
	"	 float disp = (osg_SimulationTime - startTime)*osgOcean_InversePeriod;\n"
	"\n"
	"    vec4 v_previous = gl_Vertex;\n"
	"\n"
	"	 vec3 direction = sign(gl_Normal);\n"
	"\n"
	"	 v_previous.x = direction.x * fract( disp + gl_Vertex.x );\n"
	"	 v_previous.y = direction.y * fract( disp + gl_Vertex.y );\n"
	"	 v_previous.z = direction.z * fract( disp + gl_Vertex.z );\n"
	"\n"
	"    vec4 v_current =  v_previous;\n"
	"\n"
	"	 v_current.x += ( osg_DeltaSimulationTime * osgOcean_InversePeriod );\n"
	"	 v_current.y += ( osg_DeltaSimulationTime * osgOcean_InversePeriod );\n"
	"	 v_current.z += ( osg_DeltaSimulationTime * osgOcean_InversePeriod );\n"
	"\n"
	"    colour = osgOcean_ParticleColour;\n"
	"\n"
	"    vec4 v1 = gl_ModelViewMatrix * v_current;\n"
	"    vec4 v2 = gl_ModelViewMatrix * v_previous;\n"
	"\n"
	"    vec3 dv = v2.xyz - v1.xyz;\n"
	"\n"
	"    vec2 dv_normalized = normalize(dv.xy);\n"
	"    dv.xy += dv_normalized * osgOcean_ParticleSize;\n"
	"    vec2 dp = vec2( -dv_normalized.y, dv_normalized.x ) * osgOcean_ParticleSize;\n"
	"\n"
	"    float area = length(dv.xy);\n"
	"    colour.a = 0.05+(osgOcean_ParticleSize)/area;\n"
	"\n"
	"    v1.xyz += dv*texCoord.y;\n"
	"    v1.xy += dp*texCoord.x;\n"
	"\n"
	"    gl_Position = gl_ProjectionMatrix * v1;\n"
	"	 gl_Position.z = 0.01;\n"
	"    gl_ClipVertex = v1;\n"
	"}\n";
