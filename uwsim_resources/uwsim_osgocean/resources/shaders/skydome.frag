uniform samplerCube uEnvironmentMap;

varying vec3 vTexCoord;

void main(void)
{
   vec3 texcoord = vec3(vTexCoord.x, vTexCoord.y, -vTexCoord.z);
   gl_FragData[0] = textureCube( uEnvironmentMap, texcoord.xzy );
   gl_FragData[0].a = 0.0;
   gl_FragData[1] = vec4(0.0);
}