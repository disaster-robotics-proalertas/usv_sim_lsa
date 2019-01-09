uniform sampler2DRect osgOcean_GlareTexture;
uniform float osgOcean_GlareThreshold;

void main( void )
{
	vec2 texCoordSample = vec2(0.0);

	texCoordSample.x = gl_TexCoord[0].x - 1;
	texCoordSample.y = gl_TexCoord[0].y + 1;
	vec4 color = texture2DRect(osgOcean_GlareTexture, texCoordSample);

	texCoordSample.x = gl_TexCoord[0].x + 1;
	texCoordSample.y = gl_TexCoord[0].y + 1;
	color += texture2DRect(osgOcean_GlareTexture, texCoordSample);

	texCoordSample.x = gl_TexCoord[0].x + 1;
	texCoordSample.y = gl_TexCoord[0].y - 1;
	color += texture2DRect(osgOcean_GlareTexture, texCoordSample);

	texCoordSample.x = gl_TexCoord[0].x - 1;
	texCoordSample.y = gl_TexCoord[0].y - 1;
	color += texture2DRect(osgOcean_GlareTexture, texCoordSample);

	color = color*0.25;

	if(color.a >= osgOcean_GlareThreshold)
		gl_FragColor = color;
	else
		gl_FragColor = vec4(0.0);
}