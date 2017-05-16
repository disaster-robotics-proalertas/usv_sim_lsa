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

static const char osgOcean_godray_screen_blend_frag[] =
	"uniform sampler2DRect osgOcean_GodRayTexture;\n"
	"\n"
	"uniform vec3  osgOcean_SunDir;\n"
	"uniform vec3  osgOcean_HGg;				// Eccentricity constants controls power of forward scattering\n"
	"uniform float osgOcean_Intensity;		// Intensity tweak for god rays\n"
	"uniform vec3  osgOcean_Eye;\n"
	"\n"
	"varying vec3 vRay;\n"
	"\n"
	"const float bias = 0.15; // used to hide aliasing\n"
	"\n"
	"// Mie phase function\n"
	"float computeMie(vec3 viewDir, vec3 sunDir)\n"
	"{\n"
	"	float num = osgOcean_HGg.x;\n"
	"	float den = (osgOcean_HGg.y - osgOcean_HGg.z*dot(sunDir, viewDir));\n"
	"	den = inversesqrt(den);\n"
	"\n"
	"	float phase = num * (den*den*den);\n"
	"\n"
	"	return phase;\n"
	"}\n"
	"\n"
	"// ----------------------------------------------\n"
	"//                Main Program\n"
	"// ----------------------------------------------\n"
	"\n"
	"void main( void )\n"
	"{\n"
	"	vec4 shafts;\n"
	"\n"
	"	// average the pixels out a little to hide aliasing\n"
	"	// TODO: Maybe try a weak blur filter\n"
	"	shafts += texture2DRect(osgOcean_GodRayTexture, gl_TexCoord[1].xy);\n"
	"	shafts += texture2DRect(osgOcean_GodRayTexture, gl_TexCoord[1].zw);\n"
	"	shafts += texture2DRect(osgOcean_GodRayTexture, gl_TexCoord[2].xy);\n"
	"	shafts += texture2DRect(osgOcean_GodRayTexture, gl_TexCoord[2].zw);\n"
	"\n"
	"	shafts /= 4.0;\n"
	"\n"
	"	vec3 rayNormalised = normalize(vRay-osgOcean_Eye);\n"
	"\n"
	"	float phase = computeMie(rayNormalised, -osgOcean_SunDir);\n"
	"\n"
	"	// Calculate final color, adding a little bias (0.15 here)\n"
	"	// to hide aliasing\n"
	"	vec3 colour = (bias+osgOcean_Intensity*shafts.rgb)*phase;\n"
	"\n"
	"	vec3 ray = ( rayNormalised + vec3(1.0) ) / 2.0;\n"
	"\n"
	"	gl_FragColor = vec4(colour, 1.0);\n"
	"}\n"
	"\n"
	"\n"
	"\n";
