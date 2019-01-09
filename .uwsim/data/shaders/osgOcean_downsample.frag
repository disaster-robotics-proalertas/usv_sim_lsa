#extension GL_ARB_texture_rectangle : enable

uniform sampler2DRect osgOcean_ColorTexture;

const vec2 s1 = vec2(-1, 1);
const vec2 s2 = vec2( 1, 1);
const vec2 s3 = vec2( 1,-1);
const vec2 s4 = vec2(-1,-1);

void main( void )
{
	vec2 texCoordSample = vec2(0.0);

	texCoordSample = gl_TexCoord[0].st + s1;
	vec4 color = texture2DRect(osgOcean_ColorTexture, texCoordSample);

	texCoordSample = gl_TexCoord[0].st + s2;
	color += texture2DRect(osgOcean_ColorTexture, texCoordSample);

	texCoordSample = gl_TexCoord[0].st + s3;
	color += texture2DRect(osgOcean_ColorTexture, texCoordSample);

	texCoordSample = gl_TexCoord[0].st + s4;
	color += texture2DRect(osgOcean_ColorTexture, texCoordSample);

	gl_FragColor = color * 0.25;
}