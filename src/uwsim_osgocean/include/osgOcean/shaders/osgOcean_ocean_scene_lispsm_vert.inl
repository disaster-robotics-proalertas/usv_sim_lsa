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

static const char osgOcean_ocean_scene_lispsm_vert[] =
	"// osgOcean Uniforms\n"
	"// -----------------\n"
	"uniform mat4 osg_ViewMatrixInverse;\n"
	"uniform float osgOcean_WaterHeight;\n"
	"uniform vec3 osgOcean_Eye;\n"
	"uniform vec3 osgOcean_UnderwaterAttenuation;\n"
	"uniform vec4 osgOcean_UnderwaterDiffuse;\n"
	"uniform bool osgOcean_EnableUnderwaterScattering;\n"
	"// -----------------\n"
	"\n"
	"varying vec3 vExtinction;\n"
	"varying vec3 vInScattering;\n"
	"\n"
	"varying vec3 vNormal;\n"
	"varying vec3 vLightDir;\n"
	"varying vec3 vEyeVec;\n"
	"varying float vWorldHeight;\n"
	"\n"
	"void computeScattering( in vec3 eye, in vec3 worldVertex, out vec3 extinction, out vec3 inScattering )\n"
	"{\n"
	"	float viewDist = length(eye-worldVertex);\n"
	"	\n"
	"	float depth = max(osgOcean_WaterHeight-worldVertex.z, 0.0);\n"
	"	\n"
	"	extinction = exp(-osgOcean_UnderwaterAttenuation*viewDist*2.0);\n"
	"\n"
	"	// Need to compute accurate kd constant.\n"
	"	// const vec3 kd = vec3(0.001, 0.001, 0.001);\n"
	"	inScattering = osgOcean_UnderwaterDiffuse.rgb * (1.0-extinction*exp(-depth*vec3(0.001)));\n"
	"}\n"
	"\n"
	"void main(void)\n"
	"{\n"
	"	gl_TexCoord[0] = gl_MultiTexCoord0;\n"
	"	gl_Position = ftransform();\n"
	"	gl_FogFragCoord = gl_Position.z;\n"
	"    vec4  ecPosition  = gl_ModelViewMatrix * gl_Vertex;\n"
	"	gl_ClipVertex = ecPosition; // for reflections\n"
	"\n"
	"	vNormal = gl_NormalMatrix * gl_Normal;\n"
	"	vLightDir = gl_LightSource[0].position.xyz;\n"
	"	vEyeVec = -vec3(gl_ModelViewMatrix*gl_Vertex);\n"
	"\n"
	"	vec4 worldVertex = (osg_ViewMatrixInverse*gl_ModelViewMatrix) * gl_Vertex;\n"
	"\n"
	"    if (osgOcean_EnableUnderwaterScattering)\n"
	"        computeScattering( osgOcean_Eye, worldVertex.xyz, vExtinction, vInScattering);\n"
	"\n"
	"	vWorldHeight = worldVertex.z;\n"
	"\n"
	"    // Generate shadow map coords\n"
	"    gl_TexCoord[7].s = dot( ecPosition, gl_EyePlaneS[7] );\n"
	"    gl_TexCoord[7].t = dot( ecPosition, gl_EyePlaneT[7] );\n"
	"    gl_TexCoord[7].p = dot( ecPosition, gl_EyePlaneR[7] );\n"
	"    gl_TexCoord[7].q = dot( ecPosition, gl_EyePlaneQ[7] );\n"
	"\n"
	"}\n";
