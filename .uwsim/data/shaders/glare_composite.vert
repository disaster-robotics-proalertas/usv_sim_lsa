void main(void)
{
	gl_TexCoord[0] = gl_MultiTexCoord0;
	gl_TexCoord[1] = gl_MultiTexCoord0 * vec4(0.25,0.25,1.0,1.0);

	gl_Position = ftransform();
}