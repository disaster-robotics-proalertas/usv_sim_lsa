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

static const char osgOcean_dof_combiner_frag[] =
	"#extension GL_ARB_texture_rectangle : enable\n"
	"\n"
	"uniform sampler2DRect osgOcean_FullColourMap;    // full resolution image\n"
	"uniform sampler2DRect osgOcean_FullDepthMap;     // full resolution depth map\n"
	"uniform sampler2DRect osgOcean_BlurMap;          // downsampled and filtered image\n"
	"\n"
	"uniform vec2 osgOcean_ScreenRes;\n"
	"uniform vec2 osgOcean_ScreenResInv;\n"
	"uniform vec2 osgOcean_LowRes;\n"
	"\n"
	"#define NUM_TAPS 4\n"
	"\n"
	"// maximum CoC radius and diameter in pixels\n"
	"const vec2 vMaxCoC = vec2(5.0,10);\n"
	"\n"
	"// scale factor for maximum CoC size on low res. image\n"
	"const float radiusScale = 0.4;\n"
	"\n"
	"// contains poisson-distributed positions on the unit circle\n"
	"vec2 poisson[8];\n"
	"\n"
	"void main(void)\n"
	"{\n"
	"    poisson[0] = vec2( 0.0,       0.0);\n"
	"    poisson[1] = vec2( 0.527837, -0.085868);\n"
	"    poisson[2] = vec2(-0.040088,  0.536087);\n"
	"    poisson[3] = vec2(-0.670445, -0.179949);\n"
	"    poisson[4] = vec2(-0.419418, -0.616039);\n"
	"    poisson[5] = vec2( 0.440453, -0.639399);\n"
	"    poisson[6] = vec2(-0.757088,  0.349334);\n"
	"    poisson[7] = vec2( 0.574619,  0.685879);\n"
	"\n"
	"    // pixel size (1/image resolution) of full resolution image\n"
	"    vec2 pixelSizeHigh = osgOcean_ScreenResInv;\n"
	"\n"
	"    // pixel size of low resolution image\n"
	"    vec2 pixelSizeLow = 4.0 * pixelSizeHigh;\n"
	"\n"
	"    vec4 color = texture2DRect( osgOcean_FullColourMap, gl_TexCoord[0] );	// fetch center tap\n"
	"    //	float centerDepth = color.a; // save its depth\n"
	"    float centerDepth = texture2DRect( osgOcean_FullDepthMap, gl_TexCoord[0] ).r; // save its depth\n"
	"\n"
	"    // convert depth into blur radius in pixels\n"
	"    float discRadius = abs(centerDepth * vMaxCoC.y - vMaxCoC.x);\n"
	"\n"
	"    // compute disc radius on low-res image\n"
	"    float discRadiusLow = discRadius * radiusScale;\n"
	"\n"
	"    vec4 colorAccum = vec4(0.0);\n"
	"    float depthAccum = 0.0;\n"
	"\n"
	"	for(int t = 0; t < NUM_TAPS; t++)\n"
	"	{\n"
	"        vec2 coordHigh = gl_TexCoord[0].st + ( osgOcean_ScreenRes * (pixelSizeHigh * poisson[t] * discRadius    ));\n"
	"        vec2 coordLow  = gl_TexCoord[1].st + ( osgOcean_LowRes *    (pixelSizeLow  * poisson[t] * discRadiusLow ));\n"
	"\n"
	"        // fetch low-res tap\n"
	"        vec4 tapLow = texture2DRect( osgOcean_BlurMap, coordLow );\n"
	"\n"
	"        // fetch high-res tap\n"
	"        vec4 tapHigh = texture2DRect( osgOcean_FullColourMap, coordHigh );\n"
	"        \n"
	"        float tapHighDepth = texture2DRect( osgOcean_FullDepthMap,  coordHigh ).r;\n"
	"\n"
	"        // put tap blurriness into [0, 1] range\n"
	"        float tapBlur = abs(tapHighDepth * 2.0 - 1.0);\n"
	"\n"
	"        // mix low- and hi-res taps based on tap blurriness\n"
	"        vec4 tapColor = mix(tapHigh, tapLow, tapBlur);\n"
	"\n"
	"        // apply leaking reduction: lower weight for taps that are\n"
	"        // closer than the center tap and in focus\n"
	"        float tapDepth = (tapHighDepth >= centerDepth) ? 1.0 : abs(tapHighDepth * 2.0 - 1.0);\n"
	"\n"
	"        // accumulate\n"
	"        colorAccum += tapColor * tapDepth;\n"
	"        depthAccum += tapDepth;\n"
	"	}\n"
	"\n"
	"	// normalize and return result\n"
	"	gl_FragColor = colorAccum / depthAccum;\n"
	"}\n"
	"\n";
