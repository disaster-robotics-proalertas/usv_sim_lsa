#extension GL_ARB_texture_rectangle : enable

uniform sampler2DRect osgOcean_FullColourMap;    // full resolution image
uniform sampler2DRect osgOcean_FullDepthMap;     // full resolution depth map
uniform sampler2DRect osgOcean_BlurMap;          // downsampled and filtered image

uniform vec2 osgOcean_ScreenRes;
uniform vec2 osgOcean_ScreenResInv;
uniform vec2 osgOcean_LowRes;

#define NUM_TAPS 4

// maximum CoC radius and diameter in pixels
const vec2 vMaxCoC = vec2(5.0,10);

// scale factor for maximum CoC size on low res. image
const float radiusScale = 0.4;

// contains poisson-distributed positions on the unit circle
vec2 poisson[8];

void main(void)
{
    poisson[0] = vec2( 0.0,       0.0);
    poisson[1] = vec2( 0.527837, -0.085868);
    poisson[2] = vec2(-0.040088,  0.536087);
    poisson[3] = vec2(-0.670445, -0.179949);
    poisson[4] = vec2(-0.419418, -0.616039);
    poisson[5] = vec2( 0.440453, -0.639399);
    poisson[6] = vec2(-0.757088,  0.349334);
    poisson[7] = vec2( 0.574619,  0.685879);

    // pixel size (1/image resolution) of full resolution image
    vec2 pixelSizeHigh = osgOcean_ScreenResInv;

    // pixel size of low resolution image
    vec2 pixelSizeLow = 4.0 * pixelSizeHigh;

    vec4 color = texture2DRect( osgOcean_FullColourMap, gl_TexCoord[0] );	// fetch center tap
    //	float centerDepth = color.a; // save its depth
    float centerDepth = texture2DRect( osgOcean_FullDepthMap, gl_TexCoord[0] ).r; // save its depth

    // convert depth into blur radius in pixels
    float discRadius = abs(centerDepth * vMaxCoC.y - vMaxCoC.x);

    // compute disc radius on low-res image
    float discRadiusLow = discRadius * radiusScale;

    vec4 colorAccum = vec4(0.0);
    float depthAccum = 0.0;

	for(int t = 0; t < NUM_TAPS; t++)
	{
        vec2 coordHigh = gl_TexCoord[0].st + ( osgOcean_ScreenRes * (pixelSizeHigh * poisson[t] * discRadius    ));
        vec2 coordLow  = gl_TexCoord[1].st + ( osgOcean_LowRes *    (pixelSizeLow  * poisson[t] * discRadiusLow ));

        // fetch low-res tap
        vec4 tapLow = texture2DRect( osgOcean_BlurMap, coordLow );

        // fetch high-res tap
        vec4 tapHigh = texture2DRect( osgOcean_FullColourMap, coordHigh );
        
        float tapHighDepth = texture2DRect( osgOcean_FullDepthMap,  coordHigh ).r;

        // put tap blurriness into [0, 1] range
        float tapBlur = abs(tapHighDepth * 2.0 - 1.0);

        // mix low- and hi-res taps based on tap blurriness
        vec4 tapColor = mix(tapHigh, tapLow, tapBlur);

        // apply leaking reduction: lower weight for taps that are
        // closer than the center tap and in focus
        float tapDepth = (tapHighDepth >= centerDepth) ? 1.0 : abs(tapHighDepth * 2.0 - 1.0);

        // accumulate
        colorAccum += tapColor * tapDepth;
        depthAccum += tapDepth;
	}

	// normalize and return result
	gl_FragColor = colorAccum / depthAccum;
}

