// osgOcean uniforms
// -------------------
uniform float osgOcean_WaterHeight;
// ------------------

uniform mat4 osg_ViewMatrixInverse;
uniform mat4 osg_ViewMatrix;

varying vec4 vWorldVertex;

void main(void)
{
	// Transform the vertex into world space
	vWorldVertex = (osg_ViewMatrixInverse * gl_ModelViewMatrix) * gl_Vertex;
	vWorldVertex.xyzw /= vWorldVertex.w;

	// Project the vertex onto the ocean plane
	vec4 projectedVertex = vWorldVertex;
	projectedVertex.z = osgOcean_WaterHeight;

	gl_Position = (gl_ProjectionMatrix * osg_ViewMatrix) * projectedVertex;

	return;
}
