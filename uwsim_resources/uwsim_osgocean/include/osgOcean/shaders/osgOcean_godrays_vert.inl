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

static const char osgOcean_godrays_vert[] =
	"const int NUM_WAVES = 16;\n"
	"\n"
	"uniform vec3 osgOcean_Origin;                        // central position of vertices - sun position on water surface\n"
	"uniform vec3 osgOcean_Extinction_c;                // extinction coefficient\n"
	"uniform vec3 osgOcean_Eye;                            // Eye position in world space\n"
	"uniform vec3 osgOcean_SunDir;                        // sunlight direction\n"
	"uniform float osgOcean_Spacing;                    // spacing between vertices\n"
	"uniform float osgOcean_Waves[NUM_WAVES * 5];    // wave constants\n"
	"\n"
	"varying vec3 vIntensity;\n"
	"\n"
	"float fastFresnel(vec3 I, vec3 N, float r0)\n"
	"{\n"
	"    return r0 + (1.0-r0) * pow(1.0-dot(I, N), 5.0);\n"
	"}\n"
	"\n"
	"vec3 calculateWaterNormal(float x0, float y0)\n"
	"{\n"
	"    vec3 t1 = vec3(1.0,0.0,0.0);\n"
	"    vec3 t2 = vec3(0.0,1.0,0.0);\n"
	"\n"
	"    int itr = NUM_WAVES/4;\n"
	"\n"
	"    for (int i = 0, j = 0; i < itr; i++, j += 20)\n"
	"    {\n"
	"        vec4 kx    = vec4( osgOcean_Waves[j+0],  osgOcean_Waves[j+1],  osgOcean_Waves[j+2],  osgOcean_Waves[j+3] );\n"
	"        vec4 ky    = vec4( osgOcean_Waves[j+4],  osgOcean_Waves[j+5],  osgOcean_Waves[j+6],  osgOcean_Waves[j+7] );\n"
	"        vec4 Ainvk = vec4( osgOcean_Waves[j+8],  osgOcean_Waves[j+9],  osgOcean_Waves[j+10], osgOcean_Waves[j+11] );\n"
	"        vec4 A     = vec4( osgOcean_Waves[j+12], osgOcean_Waves[j+13], osgOcean_Waves[j+14], osgOcean_Waves[j+15] );\n"
	"        vec4 wt    = vec4( osgOcean_Waves[j+16], osgOcean_Waves[j+17], osgOcean_Waves[j+18], osgOcean_Waves[j+19] );\n"
	"        vec4 phase = (kx*x0 + ky*y0 - wt);\n"
	"        vec4 sinp, cosp;\n"
	"\n"
	"#if 1\n"
	"        sinp = sin(phase);\n"
	"        cosp = cos(phase);\n"
	"#else\n"
	"        sincos(phase, sinp, cosp);\n"
	"#endif\n"
	"\n"
	"        // Update tangent vector along x0\n"
	"        t1.x -= dot(Ainvk, kx*cosp*kx);\n"
	"        t1.y -= dot(Ainvk, ky*cosp*kx);\n"
	"        t1.z += dot(A, (-sinp)*(kx));\n"
	"\n"
	"        // Update tangent vector along y0\n"
	"        t2.x -= dot(Ainvk, kx*cosp*ky);\n"
	"        t2.y -= dot(Ainvk, ky*cosp*ky);\n"
	"        t2.z += dot(A, (-sinp)*(ky));\n"
	"    }\n"
	"\n"
	"    // Calculate and return normal\n"
	"    return normalize( cross(t1, t2) );\n"
	"}\n"
	"\n"
	"void main(void)\n"
	"{\n"
	"    gl_TexCoord[0] = gl_MultiTexCoord0;\n"
	"\n"
	"    // Scale and translate the vertex on the water surface\n"
	"    vec3 worldPos = gl_Vertex.xyz * vec3(osgOcean_Spacing,osgOcean_Spacing,1.0);\n"
	"    worldPos += osgOcean_Origin;\n"
	"\n"
	"    // Calculate the water normal at this point\n"
	"    vec3 normal = calculateWaterNormal(worldPos.x, worldPos.y);\n"
	"\n"
	"    // Calculate transmittance\n"
	"    // BUG: makes intensity too small not sure why.\n"
	"    float transmittance = 1.0-fastFresnel(-osgOcean_SunDir, normal, 0.0204);\n"
	"\n"
	"    // Extrude bottom vertices along the direction of the refracted\n"
	"    // sunlight\n"
	"    if (gl_TexCoord[0].s > 0.0)\n"
	"    {\n"
	"        // Calculate refraction vector and extrude polygon\n"
	"        vec3 refr = refract(osgOcean_SunDir, normal, 0.75);\n"
	"        worldPos += refr*gl_TexCoord[0].s;\n"
	"    }\n"
	"    // Set intensity so that the further away you go from the surface\n"
	"    float totalDist = gl_TexCoord[0].s + length(worldPos-osgOcean_Eye);\n"
	"    vIntensity = exp(-totalDist*osgOcean_Extinction_c)*transmittance;\n"
	"    vIntensity = clamp(vIntensity, 0.0, 0.06);\n"
	"\n"
	"    // Transform position from world to clip space\n"
	"    gl_Position = gl_ModelViewProjectionMatrix * vec4(worldPos, 1.0 );\n"
	"    // Tweak z position not to clip shafts very close to the viewer\n"
	"    gl_Position.z = 0.01;\n"
	"}\n";
