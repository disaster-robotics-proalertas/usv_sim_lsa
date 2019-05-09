uniform sampler2DRect osgOcean_FullColourMap;    // full resolution image
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

	float discRadius, discRadiusLow, centerDepth;

	// pixel size (1/image resolution) of full resolution image
	vec2 pixelSizeHigh = osgOcean_ScreenResInv;

	// pixel size of low resolution image
	vec2 pixelSizeLow = 4.0 * pixelSizeHigh;

	vec4 color = texture2DRect( osgOcean_FullColourMap, gl_TexCoord[0] );	// fetch center tap
	centerDepth = color.a; // save its depth

	// convert depth into blur radius in pixels
	discRadius = abs(color.a * vMaxCoC.y - vMaxCoC.x);

	// compute disc radius on low-res image
	discRadiusLow = discRadius * radiusScale;

	// reuse color as an accumulator
	color = vec4(0.0);

	for(int t = 0; t < NUM_TAPS; t++)
	{
		// fetch low-res tap
		vec2 coordLow = gl_TexCoord[1].st + ( osgOcean_LowRes * (pixelSizeLow * poisson[t] * discRadiusLow) );
		vec4 tapLow = texture2DRect(osgOcean_BlurMap, coordLow);

		// fetch high-res tap
		vec2 coordHigh = gl_TexCoord[0].st + ( osgOcean_ScreenRes * (pixelSizeHigh * poisson[t] * discRadius) );
		vec4 tapHigh = texture2DRect(osgOcean_FullColourMap, coordHigh);

		// put tap blurriness into [0, 1] range
		float tapBlur = abs(tapHigh.a * 2.0 - 1.0);

		// mix low- and hi-res taps based on tap blurriness
		vec4 tap = mix(tapHigh, tapLow, tapBlur);

		// apply leaking reduction: lower weight for taps that are
		// closer than the center tap and in focus
		tap.a = (tap.a >= centerDepth) ? 1.0 : abs(tap.a * 2.0 - 1.0);

		// accumulate
		color.rgb += tap.rgb * tap.a;
		color.a += tap.a;
	}

	// normalize and return result
	gl_FragColor = color / color.a;
}

