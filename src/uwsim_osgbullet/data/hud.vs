uniform int texwidth;
uniform int osg_FrameNumber;

void main()
{
    // Incoming xy in range 0 to 1, put in range -1 to 1.
    // To do this, scale xy by 2 then bias by -1. No need for modelview and projection matrices.
    // Z is irrelevant (depth test set to GL_ALWAYS).
    vec4 pos = gl_Vertex;
    pos.x = gl_Vertex.x * 2. - 1.;
    pos.y = gl_Vertex.y * 2. - 1.;
    gl_Position = pos;

    // Modify the texture coordinates to scroll the graph.
    // The host sets s wrap to repeat.
    vec4 tc = gl_MultiTexCoord0;
    float n = float(osg_FrameNumber);
    float texw = float(texwidth);
    tc.s = gl_MultiTexCoord0.s + fract( n / texw );
    gl_TexCoord[0] = tc;
}
