uniform vec3 osgOcean_Origin;
uniform vec3 osgOcean_Extinction_c;
uniform vec3 osgOcean_Eye;
uniform float osgOcean_Spacing;

varying vec3 vIntensity;

void main(void)
{
    gl_TexCoord[0] = gl_MultiTexCoord0;

    vec3 worldPos = gl_Vertex.xyz * vec3(osgOcean_Spacing,osgOcean_Spacing,1.0);
    worldPos += osgOcean_Origin;

    vec3 extinct = vec3(0.2,0.2,0.2);

    float totalDist = length(worldPos-osgOcean_Eye)/3.0;
    vIntensity = exp(-totalDist*osgOcean_Extinction_c);
    vIntensity = clamp(vIntensity, 0.0,  1.0);

    gl_Position = gl_ModelViewProjectionMatrix * vec4(worldPos,1.0);
}
