varying vec3 vRay;

void main( void )
{
	gl_Position = gl_Vertex;

	gl_TexCoord[0] = gl_MultiTexCoord0;

	gl_TexCoord[1].xy = gl_TexCoord[0].st;
	gl_TexCoord[1].zw = gl_TexCoord[0].st + vec2(1.0, 0.0);
	gl_TexCoord[2].xy = gl_TexCoord[0].st + vec2(1.0, 1.0);
	gl_TexCoord[2].zw = gl_TexCoord[0].st + vec2(0.0, 1.0);

	vRay = gl_Normal;
}