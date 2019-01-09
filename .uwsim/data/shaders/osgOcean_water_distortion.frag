// Based on Jon Kennedy's heat haze shader
// Copyright (c) 2002-2006 3Dlabs Inc. Ltd.

uniform float osgOcean_Frequency;
uniform float osgOcean_Offset;
uniform float osgOcean_Speed;
uniform vec2  osgOcean_ScreenRes;

uniform sampler2DRect osgOcean_FrameBuffer;

varying vec4 vEyePos;

void main (void)
{
	vec2 index;

	// perform the div by w to put the texture into screen space
	float recipW = 1.0 / vEyePos.w;
	vec2 eye = vEyePos.xy * vec2(recipW);

	float blend = max(1.0 - eye.y, 0.0);

	// calc the wobble
	// index.s = eye.x ;
	index.s = eye.x + blend * sin( osgOcean_Frequency * 5.0 * eye.x + osgOcean_Offset * osgOcean_Speed ) * 0.004;
	index.t = eye.y + blend * sin( osgOcean_Frequency * 5.0 * eye.y + osgOcean_Offset * osgOcean_Speed ) * 0.004;

	// scale and shift so we're in the range 0-1
	index.s = index.s * 0.5 + 0.5;
	index.t = index.t * 0.5 + 0.5;

	vec2 recipRes = vec2(1.0/osgOcean_ScreenRes.x, 1.0/osgOcean_ScreenRes.y);

	index.s = clamp(index.s, 0.0, 1.0 - recipRes.x);
	index.t = clamp(index.t, 0.0, 1.0 - recipRes.y);

	// scale the texture so we just see the rendered framebuffer
	index.s = index.s * osgOcean_ScreenRes.x;
	index.t = index.t * osgOcean_ScreenRes.y;

	vec3 RefractionColor = vec3( texture2DRect( osgOcean_FrameBuffer, index ) );

	gl_FragColor = vec4( RefractionColor, 1.0 );
	//gl_FragColor = texture2DRect( osgOcean_FrameBuffer, gl_TexCoord[0].st );
}