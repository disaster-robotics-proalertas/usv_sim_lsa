uniform vec4 foreground;
uniform vec4 background;
uniform vec4 overrun;
uniform sampler1D texVal;

void main()
{
    // Look up the value for this vertical bar.
    vec4 samp = texture1D( texVal, gl_TexCoord[ 0 ].s );
    float value = samp.r;

    // Color code this vertical bar based on the retrieved value.
    // If texture t > sampled value, draw in background color.
    // If texture t < sampled value, dtaw in foreground color.
    vec4 color;
    if( value > 1. )
        // Handle overrun case.
        color = overrun;
    else if( gl_TexCoord[ 0 ].t > value )
        color = background;
    else
        color = foreground;
    gl_FragColor = color;
}
