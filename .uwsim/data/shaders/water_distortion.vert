varying vec4 vEyePos;

void main(void)
{
    gl_TexCoord[0] = gl_MultiTexCoord0;
	vEyePos = gl_ModelViewProjectionMatrix * gl_Vertex;
	gl_Position = ftransform();
}