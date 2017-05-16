// osgOcean uniforms
// -------------------
uniform float osgOcean_WaterHeight;
uniform vec3 osgOcean_Eye;
uniform vec3 osgOcean_UnderwaterAttenuation;
uniform vec4 osgOcean_UnderwaterDiffuse;
uniform bool osgOcean_EnableUnderwaterScattering;
// ------------------

uniform vec2 uHeightRange;
uniform float uOneOverHeight;
uniform mat4 osg_ViewMatrixInverse;

varying float vUnitHeight;

varying vec3 vLightDir;
varying vec3 vEyeVec;
varying vec3 vExtinction;
varying vec3 vInScattering;
varying vec4 vWorldVertex;

attribute vec3 aTangent;

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
	gl_TexCoord[0] = gl_MultiTexCoord0*vec4(16.0,16.0,1.0,1.0);
	gl_TexCoord[1] = gl_MultiTexCoord0*vec4(20.0,20.0,1.0,1.0);
	gl_TexCoord[2] = gl_MultiTexCoord0*vec4(25.0,25.0,1.0,1.0);
	
    vec4 ecPosition  = gl_ModelViewMatrix * gl_Vertex;
	vec3 vertex = vec3(ecPosition);
	vEyeVec = -vertex;

	vec3 n = normalize(gl_NormalMatrix * gl_Normal);
	vec3 t = normalize(gl_NormalMatrix * aTangent);
	vec3 b = cross(n, t);

	vec3 tmpVec = vec3(gl_LightSource[0].position.xyz);
	tmpVec = normalize(tmpVec);

	vLightDir.x = dot(tmpVec, t);
	vLightDir.y = dot(tmpVec, b);
	vLightDir.z = dot(tmpVec, n);

	tmpVec = -vertex;
	vEyeVec.x = dot(tmpVec, t);
	vEyeVec.y = dot(tmpVec, b);
	vEyeVec.z = dot(tmpVec, n);

	gl_Position = ftransform();

	gl_ClipVertex = ecPosition;

	gl_FogFragCoord = gl_Position.z;

	float inv = 1.0 / ( uHeightRange.y - (uHeightRange.x+65.0) );
	vUnitHeight = inv * (gl_Vertex.z - (uHeightRange.x+65.0));

	vWorldVertex = (osg_ViewMatrixInverse*gl_ModelViewMatrix) * gl_Vertex;
	
    if( osgOcean_EnableUnderwaterScattering )
    {
	    computeScattering( osgOcean_Eye, vWorldVertex.xyz, vExtinction, vInScattering );
    }

    // Generate shadow map coords
    gl_TexCoord[7].s = dot( ecPosition, gl_EyePlaneS[7] );
    gl_TexCoord[7].t = dot( ecPosition, gl_EyePlaneT[7] );
    gl_TexCoord[7].p = dot( ecPosition, gl_EyePlaneR[7] );
    gl_TexCoord[7].q = dot( ecPosition, gl_EyePlaneQ[7] );

	gl_Position = ftransform();
}