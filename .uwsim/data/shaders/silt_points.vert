uniform float osgOcean_InversePeriod;
uniform vec4 osgOcean_ParticleColour;
uniform float osgOcean_ParticleSize;

uniform float osg_SimulationTime;

varying vec4 colour;

void main(void)
{
	float startTime = gl_MultiTexCoord1.x;

	vec4 v_current = gl_Vertex;

	float disp = (osg_SimulationTime - startTime)*osgOcean_InversePeriod;

	vec3 direction = sign(gl_Normal);

	v_current.x = direction.x * fract( disp + gl_Vertex.x );
	v_current.y = direction.y * fract( disp + gl_Vertex.y );
	v_current.z = direction.z * fract( disp + gl_Vertex.z );

	colour = osgOcean_ParticleColour;

	gl_Position = gl_ModelViewProjectionMatrix * v_current;

	float pointSize = abs(1280.0*osgOcean_ParticleSize / gl_Position.w);

	gl_PointSize = ceil(pointSize);

	colour.a = 0.05+(pointSize*pointSize)/(gl_PointSize*gl_PointSize);

	gl_ClipVertex = v_current;
}