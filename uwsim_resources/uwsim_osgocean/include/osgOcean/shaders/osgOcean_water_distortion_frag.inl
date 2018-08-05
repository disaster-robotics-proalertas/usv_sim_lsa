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

static const char osgOcean_water_distortion_frag[] =
	"// Based on Jon Kennedy's heat haze shader\n"
	"// Copyright (c) 2002-2006 3Dlabs Inc. Ltd.\n"
	"\n"
	"uniform float osgOcean_Frequency;\n"
	"uniform float osgOcean_Offset;\n"
	"uniform float osgOcean_Speed;\n"
	"uniform vec2  osgOcean_ScreenRes;\n"
	"\n"
	"uniform sampler2DRect osgOcean_FrameBuffer;\n"
	"\n"
	"varying vec4 vEyePos;\n"
	"\n"
	"void main (void)\n"
	"{\n"
	"	vec2 index;\n"
	"\n"
	"	// perform the div by w to put the texture into screen space\n"
	"	float recipW = 1.0 / vEyePos.w;\n"
	"	vec2 eye = vEyePos.xy * vec2(recipW);\n"
	"\n"
	"	float blend = max(1.0 - eye.y, 0.0);\n"
	"\n"
	"	// calc the wobble\n"
	"	// index.s = eye.x ;\n"
	"	index.s = eye.x + blend * sin( osgOcean_Frequency * 5.0 * eye.x + osgOcean_Offset * osgOcean_Speed ) * 0.004;\n"
	"	index.t = eye.y + blend * sin( osgOcean_Frequency * 5.0 * eye.y + osgOcean_Offset * osgOcean_Speed ) * 0.004;\n"
	"\n"
	"	// scale and shift so we're in the range 0-1\n"
	"	index.s = index.s * 0.5 + 0.5;\n"
	"	index.t = index.t * 0.5 + 0.5;\n"
	"\n"
	"	vec2 recipRes = vec2(1.0/osgOcean_ScreenRes.x, 1.0/osgOcean_ScreenRes.y);\n"
	"\n"
	"	index.s = clamp(index.s, 0.0, 1.0 - recipRes.x);\n"
	"	index.t = clamp(index.t, 0.0, 1.0 - recipRes.y);\n"
	"\n"
	"	// scale the texture so we just see the rendered framebuffer\n"
	"	index.s = index.s * osgOcean_ScreenRes.x;\n"
	"	index.t = index.t * osgOcean_ScreenRes.y;\n"
	"\n"
	"	vec3 RefractionColor = vec3( texture2DRect( osgOcean_FrameBuffer, index ) );\n"
	"\n"
	"	gl_FragColor = vec4( RefractionColor, 1.0 );\n"
	"	//gl_FragColor = texture2DRect( osgOcean_FrameBuffer, gl_TexCoord[0].st );\n"
	"}\n";
