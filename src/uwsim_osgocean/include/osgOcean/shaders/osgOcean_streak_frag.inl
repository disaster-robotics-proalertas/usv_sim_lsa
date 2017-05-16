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

static const char osgOcean_streak_frag[] =
	"#extension GL_ARB_texture_rectangle : enable\n"
	"\n"
	"#define NUM_SAMPLES 4\n"
	"\n"
	"uniform sampler2DRect osgOcean_Buffer;\n"
	"uniform vec2        osgOcean_Direction;\n"
	"uniform float       osgOcean_Attenuation;\n"
	"uniform float       osgOcean_Pass;\n"
	"\n"
	"void main(void)\n"
	"{\n"
	"	vec2 sampleCoord = vec2(0.0);\n"
	"	vec3 cOut = vec3(0.0);\n"
	"\n"
	"	// sample weight = a^(b*s)\n"
	"	// a = attenuation\n"
	"	// b = 4^(pass-1)\n"
	"	// s = sample number\n"
	"\n"
	"	vec2 pxSize = vec2(0.5);\n"
	"\n"
	"	float b = pow( float(NUM_SAMPLES), float(osgOcean_Pass));\n"
	"	float sf = 0.0;\n"
	"\n"
	"	for (int s = 0; s < NUM_SAMPLES; s++)\n"
	"	{\n"
	"		sf = float(s);\n"
	"		float weight = pow(osgOcean_Attenuation, b * sf);\n"
	"		sampleCoord = gl_TexCoord[0].st + (osgOcean_Direction * b * vec2(sf) * pxSize);\n"
	"		cOut += clamp(weight,0.0,1.0) * texture2DRect(osgOcean_Buffer, sampleCoord).rgb;\n"
	"	}\n"
	"\n"
	"	vec3 streak = clamp(cOut, 0.0, 1.0);\n"
	"\n"
	"	gl_FragColor = vec4(streak,1.0);\n"
	"}\n";
