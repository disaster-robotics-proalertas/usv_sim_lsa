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

vec4 lighting( vec4 colormap )
{
	vec4 final_color = gl_LightSource[osgOcean_LightID].ambient * colormap;

	vec3 N = normalize(vNormal);
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
    vec4 final_color;

    vec4 textureColor = texture2D( uTextureMap, gl_TexCoord[0].st );

    // Underwater
    // +2 tweak here as waves peak above average wave height,
    // and surface fog becomes visible.
    if(osgOcean_EyeUnderwater && vWorldHeight < osgOcean_WaterHeight+2.0 )
    {
        final_color = lighting( textureColor );

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
        gl_FragData[0] = mix( osgOcean_UnderwaterFogColor, final_color, fogFactor );
    }
    // Above water
    else
    {
        final_color = lighting( textureColor );

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

        // write to color buffer
        gl_FragData[0] = final_color;
    }
}
