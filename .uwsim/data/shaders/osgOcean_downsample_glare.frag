#extension GL_ARB_texture_rectangle : enable

uniform sampler2DRect osgOcean_ColorTexture;
uniform sampler2DRect osgOcean_LuminanceTexture;
uniform float osgOcean_GlareThreshold;

const vec2 s1 = vec2(-1, 1);
const vec2 s2 = vec2( 1, 1);
const vec2 s3 = vec2( 1,-1);
const vec2 s4 = vec2(-1,-1);

void main( void )
{
	vec2 texCoordSample = vec2(0.0);

    texCoordSample = gl_TexCoord[0].st + s1;
	vec4 color = texture2DRect(osgOcean_ColorTexture, texCoordSample);
	float lum  = texture2DRect(osgOcean_LuminanceTexture, texCoordSample).r;

	texCoordSample = gl_TexCoord[0].st + s2;
	color += texture2DRect(osgOcean_ColorTexture, texCoordSample);
    lum   += texture2DRect(osgOcean_LuminanceTexture, texCoordSample).r;

	texCoordSample = gl_TexCoord[0].st + s3;
	color += texture2DRect(osgOcean_ColorTexture, texCoordSample);
    lum   += texture2DRect(osgOcean_LuminanceTexture, texCoordSample).r;

	texCoordSample = gl_TexCoord[0].st +s4;
	color += texture2DRect(osgOcean_ColorTexture, texCoordSample);
    lum   += texture2DRect(osgOcean_LuminanceTexture, texCoordSample).r;

	color = color*0.25;
    lum = lum*0.25;

    // only want very high luminance values to pass otherwise
    // we get streaks all over the scene
	if(lum >= osgOcean_GlareThreshold)
		gl_FragColor = color;
	else
		gl_FragColor = vec4(0.0);
}