// osgOcean uniforms
// -------------------
uniform float osgOcean_WaterHeight;
// -------------------

varying vec4 vWorldVertex;

void main(void)
{
	// Store the water depth
	// maximum possible depth is 500,
	// (a higher water depth value would not have a visible effect anyway)
	gl_FragDepth = clamp((osgOcean_WaterHeight - vWorldVertex.z) / 500.0, 0.0, 1.0);

	return;
}