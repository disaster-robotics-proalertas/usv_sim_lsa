// osgOcean Uniforms
// -----------------
uniform mat4 osg_ViewMatrixInverse;
uniform float osgOcean_WaterHeight;
uniform vec3 osgOcean_Eye;
uniform vec3 osgOcean_UnderwaterAttenuation;
uniform vec4 osgOcean_UnderwaterDiffuse;
uniform bool osgOcean_EnableUnderwaterScattering;
// -----------------

varying vec3 vExtinction;
varying vec3 vInScattering;

varying vec3 vNormal;
varying vec3 vLightDir;
varying vec3 vEyeVec;
varying float vWorldHeight;

void computeScattering( in vec3 eye, in vec3 worldVertex, out vec3 extinction, out vec3 inScattering )
{
	float viewDist = length(eye-worldVertex);
	
	float depth = max(osgOcean_WaterHeight-worldVertex.z, 0.0);
	
	extinction = exp(-osgOcean_UnderwaterAttenuation*viewDist*2.0);

	// Need to compute accurate kd constant.
	// const vec3 kd = vec3(0.001, 0.001, 0.001);
	inScattering = osgOcean_UnderwaterDiffuse.rgb * (1.0-extinction*exp(-depth*vec3(0.001)));
}

void main(void)
{
	gl_TexCoord[0] = gl_MultiTexCoord0;
	gl_Position = ftransform();
	gl_FogFragCoord = gl_Position.z;
    vec4  ecPosition  = gl_ModelViewMatrix * gl_Vertex;
	gl_ClipVertex = ecPosition; // for reflections

	vNormal = gl_NormalMatrix * gl_Normal;
	vLightDir = gl_LightSource[0].position.xyz;
	vEyeVec = -vec3(gl_ModelViewMatrix*gl_Vertex);

	vec4 worldVertex = (osg_ViewMatrixInverse*gl_ModelViewMatrix) * gl_Vertex;

    if (osgOcean_EnableUnderwaterScattering)
        computeScattering( osgOcean_Eye, worldVertex.xyz, vExtinction, vInScattering);

	vWorldHeight = worldVertex.z;

    // Generate shadow map coords
    gl_TexCoord[7].s = dot( ecPosition, gl_EyePlaneS[7] );
    gl_TexCoord[7].t = dot( ecPosition, gl_EyePlaneT[7] );
    gl_TexCoord[7].p = dot( ecPosition, gl_EyePlaneR[7] );
    gl_TexCoord[7].q = dot( ecPosition, gl_EyePlaneQ[7] );

}