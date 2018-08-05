float DynamicShadow();

varying vec4 colorAmbientEmissive;

uniform sampler2D baseTexture;

void main( void )
{
    vec4 color = texture2D( baseTexture, gl_TexCoord[0].xy );
    float s = DynamicShadow();
    color *= mix( colorAmbientEmissive, gl_Color, s );
    if( s >= 1.0 )
        color += gl_SecondaryColor;
    gl_FragColor = color;
}
