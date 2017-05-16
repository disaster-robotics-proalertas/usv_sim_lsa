// osgOcean uniforms
// -------------------
uniform float osgOcean_DOF_Near;
uniform float osgOcean_DOF_Focus;
uniform float osgOcean_DOF_Far;
uniform float osgOcean_DOF_Clamp;

uniform bool osgOcean_EnableGlare;
uniform bool osgOcean_EnableDOF;
uniform bool osgOcean_EyeUnderwater;
uniform bool osgOcean_EnableUnderwaterScattering;

uniform float osgOcean_UnderwaterFogDensity;
uniform float osgOcean_AboveWaterFogDensity;
uniform float osgOcean_WaterHeight;

uniform vec4 osgOcean_UnderwaterFogColor;
uniform vec4 osgOcean_AboveWaterFogColor;
// -------------------

uniform sampler2D uTextureMap;
uniform sampler2D uOverlayMap;
uniform sampler2D uNormalMap;

varying vec3 vLightDir;
varying vec3 vEyeVec;
varying vec3 vExtinction;
varying vec3 vInScattering;

varying vec4 vWorldVertex;

varying float vUnitHeight;

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

vec4 lighting( in vec4 colormap, in vec3 N )
{
	vec4 final_color = gl_LightSource[osgOcean_LightID].ambient * colormap;

	vec3 L = normalize(vLightDir);

	float lambertTerm = dot(N,L);

	if(lambertTerm > 0.0)
	{
		final_color += gl_LightSource[osgOcean_LightID].diffuse * lambertTerm * colormap;

		vec3 E = normalize(vEyeVec);
		vec3 R = reflect(-L, N);

		float specular = pow( max(dot(R, E), 0.0), 2.0 );

		final_color += gl_LightSource[osgOcean_LightID].specular * specular;
	}

	return final_color;
}

float computeFogFactor( float density, float fogCoord )
{
	return exp2(density * fogCoord * fogCoord );
}

void main(void)
{
	vec4 baseColor    = texture2D( uTextureMap, gl_TexCoord[0].st );
	vec4 overlayColor = texture2D( uOverlayMap, gl_TexCoord[1].st );
	vec4 bumpColor    = texture2D( uNormalMap,  gl_TexCoord[0].st );

	float unitHeight = clamp( vUnitHeight, 0.0, 1.0);
	vec4 textureColor = mix(overlayColor, baseColor, unitHeight);

	vec3 bump = (bumpColor.xyz*2.0)-1.0;

	float fogFactor = 0.0;
	vec4 fogColor;

	vec4 final_color = lighting( textureColor, bump );

	// +2 tweak here as waves peak above average wave height,
	// and surface fog becomes visible.
	if(osgOcean_EyeUnderwater && vWorldVertex.z < osgOcean_WaterHeight+2.0)
	{
		// mix in underwater light
        if( osgOcean_EnableUnderwaterScattering )
        {
		    final_color.rgb = final_color.rgb * vExtinction + vInScattering;
        }

		fogFactor = computeFogFactor( osgOcean_UnderwaterFogDensity, gl_FogFragCoord );
		fogColor = osgOcean_UnderwaterFogColor;

        // depth buffer
		if(osgOcean_EnableDOF)
        {
			float depthBlur = computeDepthBlur( gl_FogFragCoord, osgOcean_DOF_Focus, osgOcean_DOF_Near, osgOcean_DOF_Far, osgOcean_DOF_Clamp );
            gl_FragData[1] = vec4(depthBlur);
        }
	}
	else
	{
		fogFactor = computeFogFactor( osgOcean_AboveWaterFogDensity, gl_FogFragCoord );
		fogColor = osgOcean_AboveWaterFogColor;
		
        // luminance buffer
		if(osgOcean_EnableGlare)
        {
            gl_FragData[1] = vec4(0.0);
        }
	}

    // color buffer
    gl_FragData[0] = mix( fogColor, final_color, fogFactor );
}