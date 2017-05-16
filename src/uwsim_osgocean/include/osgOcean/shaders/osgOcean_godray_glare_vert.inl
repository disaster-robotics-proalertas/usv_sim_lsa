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

static const char osgOcean_godray_glare_vert[] =
	"uniform vec3 osgOcean_Origin;\n"
	"uniform vec3 osgOcean_Extinction_c;\n"
	"uniform vec3 osgOcean_Eye;\n"
	"uniform float osgOcean_Spacing;\n"
	"\n"
	"varying vec3 vIntensity;\n"
	"\n"
	"void main(void)\n"
	"{\n"
	"    gl_TexCoord[0] = gl_MultiTexCoord0;\n"
	"\n"
	"    vec3 worldPos = gl_Vertex.xyz * vec3(osgOcean_Spacing,osgOcean_Spacing,1.0);\n"
	"    worldPos += osgOcean_Origin;\n"
	"\n"
	"    vec3 extinct = vec3(0.2,0.2,0.2);\n"
	"\n"
	"    float totalDist = length(worldPos-osgOcean_Eye)/3.0;\n"
	"    vIntensity = exp(-totalDist*osgOcean_Extinction_c);\n"
	"    vIntensity = clamp(vIntensity, 0.0,  1.0);\n"
	"\n"
	"    gl_Position = gl_ModelViewProjectionMatrix * vec4(worldPos,1.0);\n"
	"}\n";
