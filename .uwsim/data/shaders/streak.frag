#define NUM_SAMPLES 4

uniform sampler2DRect osgOcean_Buffer;
uniform vec2        osgOcean_Direction;
uniform float       osgOcean_Attenuation;
uniform float       osgOcean_Pass;

void main(void)
{
	vec2 sampleCoord = vec2(0.0);
	vec3 cOut = vec3(0.0);

	// sample weight = a^(b*s)
	// a = attenuation
	// b = 4^(pass-1)
	// s = sample number

	vec2 pxSize = vec2(0.5);

	float b = pow( float(NUM_SAMPLES), float(osgOcean_Pass));
	float sf = 0.0;

	for (int s = 0; s < NUM_SAMPLES; s++)
	{
		sf = float(s);
		float weight = pow(osgOcean_Attenuation, b * sf);
		sampleCoord = gl_TexCoord[0].st + (osgOcean_Direction * b * vec2(sf) * pxSize);
		cOut += clamp(weight,0.0,1.0) * texture2DRect(osgOcean_Buffer, sampleCoord).rgb;
	}

	vec3 streak = clamp(cOut, 0.0, 1.0);

	gl_FragColor = vec4(streak,1.0);
}