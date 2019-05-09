uniform vec2 screenRes;

void main(void)
{
   gl_TexCoord[0] = gl_MultiTexCoord0 * vec4( screenRes, 1.0, 1.0 );
   gl_Position = ftransform();
}