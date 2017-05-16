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

static const char osgOcean_godray_glare_frag[] =
	"uniform sampler2D osgOcean_GlareTexture;\n"
	"\n"
	"varying vec3 vIntensity;\n"
	"\n"
	"void main(void)\n"
	"{\n"
	"    vec3 color = texture2D( osgOcean_GlareTexture, gl_TexCoord[0].st ).rgb;\n"
	"\n"
	"    gl_FragColor = vec4((vIntensity*color.r)*1.5, 1.0 );\n"
	"}\n";
