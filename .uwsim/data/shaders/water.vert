uniform mat4 osg_ViewMatrixInverse;

uniform vec3 osgOcean_EyePosition;

uniform vec3 osgOcean_NoiseCoords0;
uniform vec3 osgOcean_NoiseCoords1;

uniform vec4 osgOcean_WaveTop;
uniform vec4 osgOcean_WaveBot;

uniform float osgOcean_FoamScale;

varying vec4 vVertex;
varying vec4 vWorldVertex;
varying vec3 vNormal;
varying vec3 vViewerDir;
varying vec3 vLightDir;

varying vec3 vWorldViewDir;
varying vec3 vWorldNormal;

mat3 get3x3Matrix( mat4 m )
{
	mat3 result;

	result[0][0] = m[0][0];
	result[0][1] = m[0][1];
	result[0][2] = m[0][2];

	result[1][0] = m[1][0];
	result[1][1] = m[1][1];
	result[1][2] = m[1][2];

	result[2][0] = m[2][0];
	result[2][1] = m[2][1];
	result[2][2] = m[2][2];

	return result;
}

// -------------------------------
//          Main Program
// -------------------------------

void main( void )
{
	gl_Position = ftransform();

	// -----------------------------------------------------------

	// In object space
	vVertex = gl_Vertex;
	vLightDir = normalize( vec3( gl_ModelViewMatrixInverse * ( gl_LightSource[osgOcean_LightID].position ) ) );
	vViewerDir = gl_ModelViewMatrixInverse[3].xyz - gl_Vertex.xyz;
	vNormal = normalize(gl_Normal);

	vec4 waveColorDiff = osgOcean_WaveTop-osgOcean_WaveBot;

	gl_FrontColor = waveColorDiff *
		clamp((gl_Vertex.z + osgOcean_EyePosition.z) * 0.1111111 + vNormal.z - 0.4666667, 0.0, 1.0) + osgOcean_WaveBot;

	// -------------------------------------------------------------

	mat4 modelMatrix = osg_ViewMatrixInverse * gl_ModelViewMatrix;
	mat3 modelMatrix3x3 = get3x3Matrix( modelMatrix );

	// world space
	vWorldVertex = modelMatrix * gl_Vertex;
	vWorldNormal = modelMatrix3x3 * gl_Normal;
	vWorldViewDir = vWorldVertex.xyz - osgOcean_EyePosition.xyz;

	// ------------- Texture Coords ---------------------------------

	// Normal Map Coords
	gl_TexCoord[0].xy = ( gl_Vertex.xy * osgOcean_NoiseCoords0.z + osgOcean_NoiseCoords0.xy );
	gl_TexCoord[0].zw = ( gl_Vertex.xy * osgOcean_NoiseCoords1.z + osgOcean_NoiseCoords1.xy );
	gl_TexCoord[0].y = -gl_TexCoord[0].y;
	gl_TexCoord[0].w = -gl_TexCoord[0].w;

	// Foam coords
	gl_TexCoord[1].st = gl_Vertex.xy * osgOcean_FoamScale;

	// Fog coords
	gl_FogFragCoord = gl_Position.z;
}