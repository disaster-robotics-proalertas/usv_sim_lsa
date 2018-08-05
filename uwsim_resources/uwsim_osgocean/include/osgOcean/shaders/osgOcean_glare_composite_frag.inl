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

static const char osgOcean_glare_composite_frag[] =
	"#extension GL_ARB_texture_rectangle : enable\n"
	"\n"
	"uniform sampler2DRect osgOcean_ColorBuffer;\n"
	"uniform sampler2DRect osgOcean_StreakBuffer1;\n"
	"uniform sampler2DRect osgOcean_StreakBuffer2;\n"
	"uniform sampler2DRect osgOcean_StreakBuffer3;\n"
	"uniform sampler2DRect osgOcean_StreakBuffer4;\n"
	"\n"
	"void main(void)\n"
	"{\n"
	"	vec4 fullColor    = texture2DRect(osgOcean_ColorBuffer,   gl_TexCoord[0].st );\n"
	"	vec4 streakColor1 = texture2DRect(osgOcean_StreakBuffer1, gl_TexCoord[1].st );\n"
	"	vec4 streakColor2 = texture2DRect(osgOcean_StreakBuffer2, gl_TexCoord[1].st );\n"
	"	vec4 streakColor3 = texture2DRect(osgOcean_StreakBuffer3, gl_TexCoord[1].st );\n"
	"	vec4 streakColor4 = texture2DRect(osgOcean_StreakBuffer4, gl_TexCoord[1].st );\n"
	"\n"
	"	vec4 streak = streakColor1+streakColor2+streakColor3+streakColor4;\n"
	"\n"
	"	gl_FragColor = streak+fullColor; \n"
	"}\n";
