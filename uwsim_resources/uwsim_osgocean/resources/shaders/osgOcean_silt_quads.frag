uniform sampler2D osgOcean_BaseTexture;
varying vec2 texCoord;
varying vec4 colour;

void main (void)
{
    gl_FragColor = colour * texture2D( osgOcean_BaseTexture, texCoord);
}