const int NUM_WAVES = 16;

uniform vec3 osgOcean_Origin;                        // central position of vertices - sun position on water surface
uniform vec3 osgOcean_Extinction_c;                // extinction coefficient
uniform vec3 osgOcean_Eye;                            // Eye position in world space
uniform vec3 osgOcean_SunDir;                        // sunlight direction
uniform float osgOcean_Spacing;                    // spacing between vertices
uniform float osgOcean_Waves[NUM_WAVES * 5];    // wave constants

varying vec3 vIntensity;

float fastFresnel(vec3 I, vec3 N, float r0)
{
    return r0 + (1.0-r0) * pow(1.0-dot(I, N), 5.0);
}

vec3 calculateWaterNormal(float x0, float y0)
{
    vec3 t1 = vec3(1.0,0.0,0.0);
    vec3 t2 = vec3(0.0,1.0,0.0);

    int itr = NUM_WAVES/4;

    for (int i = 0, j = 0; i < itr; i++, j += 20)
    {
        vec4 kx    = vec4( osgOcean_Waves[j+0],  osgOcean_Waves[j+1],  osgOcean_Waves[j+2],  osgOcean_Waves[j+3] );
        vec4 ky    = vec4( osgOcean_Waves[j+4],  osgOcean_Waves[j+5],  osgOcean_Waves[j+6],  osgOcean_Waves[j+7] );
        vec4 Ainvk = vec4( osgOcean_Waves[j+8],  osgOcean_Waves[j+9],  osgOcean_Waves[j+10], osgOcean_Waves[j+11] );
        vec4 A     = vec4( osgOcean_Waves[j+12], osgOcean_Waves[j+13], osgOcean_Waves[j+14], osgOcean_Waves[j+15] );
        vec4 wt    = vec4( osgOcean_Waves[j+16], osgOcean_Waves[j+17], osgOcean_Waves[j+18], osgOcean_Waves[j+19] );
        vec4 phase = (kx*x0 + ky*y0 - wt);
        vec4 sinp, cosp;

#if 1
        sinp = sin(phase);
        cosp = cos(phase);
#else
        sincos(phase, sinp, cosp);
#endif

        // Update tangent vector along x0
        t1.x -= dot(Ainvk, kx*cosp*kx);
        t1.y -= dot(Ainvk, ky*cosp*kx);
        t1.z += dot(A, (-sinp)*(kx));

        // Update tangent vector along y0
        t2.x -= dot(Ainvk, kx*cosp*ky);
        t2.y -= dot(Ainvk, ky*cosp*ky);
        t2.z += dot(A, (-sinp)*(ky));
    }

    // Calculate and return normal
    return normalize( cross(t1, t2) );
}

void main(void)
{
    gl_TexCoord[0] = gl_MultiTexCoord0;

    // Scale and translate the vertex on the water surface
    vec3 worldPos = gl_Vertex.xyz * vec3(osgOcean_Spacing,osgOcean_Spacing,1.0);
    worldPos += osgOcean_Origin;

    // Calculate the water normal at this point
    vec3 normal = calculateWaterNormal(worldPos.x, worldPos.y);

    // Calculate transmittance
    // BUG: makes intensity too small not sure why.
    float transmittance = 1.0-fastFresnel(-osgOcean_SunDir, normal, 0.0204);

    // Extrude bottom vertices along the direction of the refracted
    // sunlight
    if (gl_TexCoord[0].s > 0.0)
    {
        // Calculate refraction vector and extrude polygon
        vec3 refr = refract(osgOcean_SunDir, normal, 0.75);
        worldPos += refr*gl_TexCoord[0].s;
    }
    // Set intensity so that the further away you go from the surface
    float totalDist = gl_TexCoord[0].s + length(worldPos-osgOcean_Eye);
    vIntensity = exp(-totalDist*osgOcean_Extinction_c)*transmittance;
    vIntensity = clamp(vIntensity, 0.0, 0.06);

    // Transform position from world to clip space
    gl_Position = gl_ModelViewProjectionMatrix * vec4(worldPos, 1.0 );
    // Tweak z position not to clip shafts very close to the viewer
    gl_Position.z = 0.01;
}
