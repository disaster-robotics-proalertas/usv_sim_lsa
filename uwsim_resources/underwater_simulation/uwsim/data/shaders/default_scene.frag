// osgOcean Uniforms
// -----------------
uniform float osgOcean_DOF_Near;
uniform float osgOcean_DOF_Focus;
uniform float osgOcean_DOF_Far;
uniform float osgOcean_DOF_Clamp;

uniform float osgOcean_UnderwaterFogDensity;
uniform float osgOcean_AboveWaterFogDensity;
uniform vec4  osgOcean_UnderwaterFogColor;
uniform vec4  osgOcean_AboveWaterFogColor;

uniform float osgOcean_WaterHeight;

uniform bool osgOcean_EnableGlare;
uniform bool osgOcean_EnableDOF;
uniform bool osgOcean_EyeUnderwater;
uniform bool osgOcean_EnableUnderwaterScattering;
// -------------------

uniform sampler2D uTextureMap;

varying vec3 vExtinction;
varying vec3 vInScattering;
varying vec3 vNormal;
varying vec3 vLightDir;
varying vec3 vEyeVec;

varying float vWorldHeight;

varying vec4 color;

//Laser variables
// -----------------
uniform bool sls_projector;

uniform sampler2D SLStex;
uniform sampler2D SLStex2;
varying vec4 ShadowCoord;
uniform bool isLaser;
// -----------------

//Noise variables
//----------------
uniform vec4 offsets;
uniform float mean;
uniform float stddev;
//----------------

//Light rate
uniform float light;
//----------------

#define PI 3.14159265358979323846264

//The two following functions were adapted from Gazebo camera_noise_gaussian_fs.glsl shader
//The main idea is to use 4 CPU random number seeds added to current pixel coordinates in a 
//pseudo-random number generator so we can generate through Box-Muller transform 4 Gaussian outputs
//More info on original Gazebo noise shader:
//https://github.com/thomas-moulard/gazebo-deb/blob/master/media/materials/programs/camera_noise_gaussian_fs.glsl

float rand(vec2 co)
{
  // This one-liner can be found in many places, including:
  // http://stackoverflow.com/questions/4200224/random-noise-functions-for-glsl
  // I can't find any explanation for it, but experimentally it does seem to
  // produce approximately uniformly distributed values in the interval [0,1].
  float r = fract(sin(dot(co.xy, vec2(12.9898,78.233))) * 43758.5453);

  // Make sure that we don't return 0.0
  if(r == 0.0)
    return 0.000000000001;
  else
    return r;
}

vec4 gaussrand(vec2 co)
{
  // Box-Muller method for sampling from the normal distribution
  // http://en.wikipedia.org/wiki/Normal_distribution#Generating_values_from_normal_distribution
  // This method requires 2 uniform random inputs and produces 2 
  // Gaussian random outputs.  We'll take a 3rd random variable and use it to
  // switch between the two outputs.

  float U, V, W, X;
  // Add in the CPU-supplied random offsets to generate the 4 random values that
  // we'll use.
  U = rand(co + vec2(offsets.x, offsets.x));
  V = rand(co + vec2(offsets.y, offsets.y));
  W = rand(co + vec2(offsets.z, offsets.z));
  X = rand(co + vec2(offsets.w, offsets.w));
  
  //generate 3 normal distributed numbers.
  float r,g,b;
  r = sqrt(-2.0 * log(U)) * sin(2.0 * PI * V);
  g = sqrt(-2.0 * log(U)) * cos(2.0 * PI * V);
  b = sqrt(-2.0 * log(W)) * sin(2.0 * PI * X);

  // Apply the stddev and mean.
  r = r * stddev + mean;
  g = g * stddev + mean;
  b = b * stddev + mean;

  // Return it as a vec4, to be added to the input ("true") color.
  return vec4(r,g,b, 0.0);
}


float computeDepthBlur(float depth, float focus, float near, float far, float clampval )
{
   float f;
   if (depth < focus){
      f = (depth - focus)/(focus - near);
   }
   else{
      f = (depth - focus)/(far - focus);
      f = clamp(f, 0.0, clampval);
   }
   return f * 0.5 + 0.5;
}

vec4 lighting( vec4 colormap , float lightRate)
{
	vec4 final_color = gl_LightSource[osgOcean_LightID].ambient * colormap * lightRate;

	vec3 N = normalize(vNormal);
	vec3 L = normalize(vLightDir);

	float lambertTerm = dot(N,L);

	if(lambertTerm > 0.0)
	{
		final_color += gl_LightSource[osgOcean_LightID].diffuse * lambertTerm * colormap * lightRate;

		vec3 E = normalize(vEyeVec);
		vec3 R = reflect(-L, N);

		float specular = pow( max(dot(R, E), 0.0), 2.0 );

		final_color += gl_LightSource[osgOcean_LightID].specular * specular * lightRate;
	}

	return final_color;
}

float computeFogFactor( float density, float fogCoord )
{
	return exp2(density * fogCoord * fogCoord );
}

void main(void)
{
    vec4 final_color;

    vec4 textureColor = texture2D( uTextureMap, gl_TexCoord[0].st ) * color;

    // set default light color (just in case light projection is used)
    vec4 lightColor = vec4(0.0,0.0,0.0,0.0);

    //Laser computation
    // -----------------

    if(sls_projector)
    {
      // CHECK Shadowed elements in laser (0.5 shadow, 1.0 clear)
      vec4 shadowCoordinateWdivide = ShadowCoord / ShadowCoord.w ;
	
      // Used to lower moiré pattern and self-shadowing
      shadowCoordinateWdivide.z -= 0.005;

      float distanceFromLight = texture2D(SLStex,shadowCoordinateWdivide.st).z;
	
      float shadow = 1.0;
      if (ShadowCoord.w > 0.0 )
      {
	shadow = distanceFromLight < shadowCoordinateWdivide.z ? 0.5 : 1.0 ;
      }
      // END CHECK Shadowed elements in laser (0.5 shadow, 1.0 clear)

      // get SLS texture
      vec4 texcolor=texture2D( SLStex2, shadowCoordinateWdivide.st );

      //check SLS texture for backprojection, shadow and out of texture bounds
      if(distanceFromLight>0.0 && ShadowCoord.w > 0.20 && shadow!=0.5 && texcolor!=vec4(1.0,1.0,1.0,1.0) && texcolor.w>0.0)
      {
        if(isLaser)//treating as laser projection (not dependent on the distance, substitutes original color)
	{ 
	  if (round(texcolor.x)+round(texcolor.y)+round(texcolor.z)>0.0)
	  {
            //Set Laser as light color and unset texture color (textureColor will suffer from lighting)
	    lightColor = vec4(round(texcolor.x),round(texcolor.y),round(texcolor.z),1.0);
            textureColor = vec4(0,0,0,0);
	  }
	}
	else //treating as light projection (dependent on the distance, added to original color)
	{
	  lightColor.w = 1.0;
	  lightColor.xyz = texcolor.xyz/(distanceFromLight*distanceFromLight);
	}
      }
    }

    // -----------------

    // Underwater
    // +2 tweak here as waves peak above average wave height,
    // and surface fog becomes visible.
    //if(osgOcean_EyeUnderwater && vWorldHeight < osgOcean_WaterHeight+2.0 )
    //JP: this tweak is highly inefficient and has really small benefits -> commented
    if(osgOcean_EyeUnderwater)
    {
        final_color = lighting( textureColor , light)+lightColor;

        // mix in underwater light
        if(osgOcean_EnableUnderwaterScattering)
        {
            final_color.rgb = final_color.rgb * vExtinction + vInScattering;
        }

        float fogFactor = computeFogFactor( osgOcean_UnderwaterFogDensity, gl_FogFragCoord );

        // write to depth buffer (actually a GL_LUMINANCE)
        if(osgOcean_EnableDOF)
        {
            float depth = computeDepthBlur(gl_FogFragCoord, osgOcean_DOF_Focus, osgOcean_DOF_Near, osgOcean_DOF_Far, osgOcean_DOF_Clamp);
            gl_FragData[1] = vec4(depth);
        }

        // color buffer
        final_color = mix( osgOcean_UnderwaterFogColor, final_color, fogFactor );
    }
    // Above water
    else
    {
        final_color = lighting( textureColor , light)+lightColor;

        float fogFactor = computeFogFactor( osgOcean_AboveWaterFogDensity, gl_FogFragCoord );
        final_color = mix( osgOcean_AboveWaterFogColor, final_color, fogFactor );

        // write to luminance buffer
        // might not need the IF here, glsl compiler doesn't complain if 
        // you try and write to a FragData index that doesn't exist. But since
        // Mac GLSL support seems so fussy I'll leave it in.
        if(osgOcean_EnableGlare)
        {
            gl_FragData[1] = vec4(0.0);
        }
    }

    // write to color buffer adding noise
    gl_FragData[0] = final_color+ gaussrand(gl_FragCoord.xy);
}
