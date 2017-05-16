varying vec3 vTexCoord;

void main(void)
{
   gl_Position = gl_ModelViewProjectionMatrix * vec4(gl_Vertex.xyz, 1.0);
   
   vTexCoord = gl_Vertex.xyz;
}