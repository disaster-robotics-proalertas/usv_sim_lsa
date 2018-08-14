uniform sampler2DRect osgOcean_GodRayTexture;

uniform vec3  osgOcean_SunDir;
uniform vec3  osgOcean_HGg;				// Eccentricity constants controls power of forward scattering
uniform float osgOcean_Intensity;		// Intensity tweak for god rays
uniform vec3  osgOcean_Eye;

varying vec3 vRay;

const float bias = 0.15; // used to hide aliasing

// Mie phase function
float computeMie(vec3 viewDir, vec3 sunDir)
{
	float num = osgOcean_HGg.x;
	float den = (osgOcean_HGg.y - osgOcean_HGg.z*dot(sunDir, viewDir));
	den = inversesqrt(den);

	float phase = num * (den*den*den);

	return phase;
}

// ----------------------------------------------
//                Main Program
// ----------------------------------------------

void main( void )
{
	vec4 shafts;

	// average the pixels out a little to hide aliasing
	// TODO: Maybe try a weak blur filter
	shafts += texture2DRect(osgOcean_GodRayTexture, gl_TexCoord[1].xy);
	shafts += texture2DRect(osgOcean_GodRayTexture, gl_TexCoord[1].zw);
	shafts += texture2DRect(osgOcean_GodRayTexture, gl_TexCoord[2].xy);
	shafts += texture2DRect(osgOcean_GodRayTexture, gl_TexCoord[2].zw);

	shafts /= 4.0;

	vec3 rayNormalised = normalize(vRay-osgOcean_Eye);

	float phase = computeMie(rayNormalised, -osgOcean_SunDir);

	// Calculate final color, adding a little bias (0.15 here)
	// to hide aliasing
	vec3 colour = (bias+osgOcean_Intensity*shafts.rgb)*phase;

	vec3 ray = ( rayNormalised + vec3(1.0) ) / 2.0;

	gl_FragColor = vec4(colour, 1.0);
}



