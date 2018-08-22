uniform vec2 osgOcean_ScreenRes;
uniform vec2 osgOcean_LowRes;

void main( void )
{
	gl_TexCoord[0] = gl_MultiTexCoord0 * vec4( osgOcean_ScreenRes, 1.0, 1.0 );
	gl_TexCoord[1] = gl_MultiTexCoord0 * vec4( osgOcean_LowRes,    1.0, 1.0 );

	gl_Position = ftransform();
}

