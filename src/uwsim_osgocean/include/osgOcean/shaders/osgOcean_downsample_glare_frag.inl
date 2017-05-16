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

static const char osgOcean_downsample_glare_frag[] =
	"#extension GL_ARB_texture_rectangle : enable\n"
	"\n"
	"uniform sampler2DRect osgOcean_ColorTexture;\n"
	"uniform sampler2DRect osgOcean_LuminanceTexture;\n"
	"uniform float osgOcean_GlareThreshold;\n"
	"\n"
	"const vec2 s1 = vec2(-1, 1);\n"
	"const vec2 s2 = vec2( 1, 1);\n"
	"const vec2 s3 = vec2( 1,-1);\n"
	"const vec2 s4 = vec2(-1,-1);\n"
	"\n"
	"void main( void )\n"
	"{\n"
	"	vec2 texCoordSample = vec2(0.0);\n"
	"\n"
	"    texCoordSample = gl_TexCoord[0].st + s1;\n"
	"	vec4 color = texture2DRect(osgOcean_ColorTexture, texCoordSample);\n"
	"	float lum  = texture2DRect(osgOcean_LuminanceTexture, texCoordSample).r;\n"
	"\n"
	"	texCoordSample = gl_TexCoord[0].st + s2;\n"
	"	color += texture2DRect(osgOcean_ColorTexture, texCoordSample);\n"
	"    lum   += texture2DRect(osgOcean_LuminanceTexture, texCoordSample).r;\n"
	"\n"
	"	texCoordSample = gl_TexCoord[0].st + s3;\n"
	"	color += texture2DRect(osgOcean_ColorTexture, texCoordSample);\n"
	"    lum   += texture2DRect(osgOcean_LuminanceTexture, texCoordSample).r;\n"
	"\n"
	"	texCoordSample = gl_TexCoord[0].st +s4;\n"
	"	color += texture2DRect(osgOcean_ColorTexture, texCoordSample);\n"
	"    lum   += texture2DRect(osgOcean_LuminanceTexture, texCoordSample).r;\n"
	"\n"
	"	color = color*0.25;\n"
	"    lum = lum*0.25;\n"
	"\n"
	"    // only want very high luminance values to pass otherwise\n"
	"    // we get streaks all over the scene\n"
	"	if(lum >= osgOcean_GlareThreshold)\n"
	"		gl_FragColor = color;\n"
	"	else\n"
	"		gl_FragColor = vec4(0.0);\n"
	"}\n";
