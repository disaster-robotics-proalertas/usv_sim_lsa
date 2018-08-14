uniform sampler2D osgOcean_GlareTexture;

varying vec3 vIntensity;

void main(void)
{
    vec3 color = texture2D( osgOcean_GlareTexture, gl_TexCoord[0].st ).rgb;

    gl_FragColor = vec4((vIntensity*color.r)*1.5, 1.0 );
}
