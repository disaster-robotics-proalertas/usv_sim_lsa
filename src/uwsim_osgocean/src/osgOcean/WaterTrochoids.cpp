/*
* This source file is part of the osgOcean library
* 
* Copyright (C) 2009 Kim Bale
* Copyright (C) 2009 The University of Hull, UK
* 
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU Lesser General Public License as published by the Free Software
* Foundation; either version 3 of the License, or (at your option) any later
* version.

* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
* http://www.gnu.org/copyleft/lesser.txt.
*/

#include <osgOcean/WaterTrochoids>

using namespace osgOcean;

WaterTrochoids::WaterTrochoids(void)
    :_amplitude    (0.1f)
    ,_amplitudeMul (0.5f)
    ,_lambda0      (14.0f)
    ,_lambdaMul    (1.2f)
    ,_direction    (1.00f)
    ,_angleDev     (0.2f)
{
}

WaterTrochoids::WaterTrochoids( float amplitude,
                                float amplitudeMul,
                                float baseWavelen,
                                float wavelenMul, 
                                float direction, 
                                float angleDev )
    :_amplitude    (amplitude)
    ,_amplitudeMul (amplitudeMul)
    ,_lambda0      (baseWavelen)
    ,_lambdaMul    (wavelenMul)
    ,_direction    (direction)
    ,_angleDev     (angleDev)
{
}

WaterTrochoids::WaterTrochoids( const WaterTrochoids& copy )
    :_amplitude    (copy._amplitude)
    ,_amplitudeMul (copy._amplitudeMul)
    ,_lambda0      (copy._lambda0)
    ,_lambdaMul    (copy._lambdaMul)
    ,_direction    (copy._direction)
    ,_angleDev     (copy._angleDev)
    ,_waves        (copy._waves)
{
}

WaterTrochoids::~WaterTrochoids(void)
{
}

void WaterTrochoids::updateWaves(float time) 
{
    for (std::vector<Wave>::iterator wave = _waves.begin();
        wave != _waves.end(); 
        ++wave ) 
    {
	    wave->phase = wave->w*time+wave->phi0;
    }
}

void WaterTrochoids::createWaves(void)
{
    const float wavesDirX = cos(_direction);
    const float wavesDirY = sin(_direction);

    _waves.resize(NUM_WAVES);

    float A = 1.0f;
    float lambda = _lambda0;

    for (int i = 0; i < NUM_WAVES; i++) 
    {
        // Randomly rotate the wave around a main direction
        float rads = _angleDev * nextRandomDouble(-1,1);
        float rx = cos(rads);
        float ry = sin(rads);

        // Initialize wave vector
        float k = 2.f*osg::PI/lambda;
        _waves[i].kx = k*(wavesDirX * rx + wavesDirY * ry);
        _waves[i].ky = k*(wavesDirX * -ry + wavesDirY * rx);
        _waves[i].kmod = k;

        // Initialize the wave amplitude
        _waves[i].A = A*_amplitude;
        _waves[i].Ainvk = _waves[i].A/_waves[i].kmod;

        // Initialize the wave frequency, using a standard formula
        _waves[i].w = sqrt(9.8f*k);

        // Initialize the wave initial phase
        _waves[i].phi0 = nextRandomDouble(0, osg::PI*2.f);

        // Move to next wave
        lambda *= _lambdaMul;
        A *= _amplitudeMul;
    }
}

void WaterTrochoids::packWaves(osg::FloatArray* constants) const 
{
    constants->resize( _waves.size() * 5);

    unsigned int ptr = 0;
    unsigned int itr = _waves.size()/4;

    for(unsigned i = 0, j = 0; i < itr; i++, j += 4) 
    {
        // kx
        (*constants)[ptr+0] = _waves[j+0].kx;
        (*constants)[ptr+1] = _waves[j+1].kx;
        (*constants)[ptr+2] = _waves[j+2].kx;
        (*constants)[ptr+3] = _waves[j+3].kx;
        ptr += 4;
        // ky
        (*constants)[ptr+0] = _waves[j+0].ky;
        (*constants)[ptr+1] = _waves[j+1].ky;
        (*constants)[ptr+2] = _waves[j+2].ky;
        (*constants)[ptr+3] = _waves[j+3].ky;
        ptr += 4;
        // amplitude / k
        (*constants)[ptr+0] = _waves[j+0].Ainvk;
        (*constants)[ptr+1] = _waves[j+1].Ainvk;
        (*constants)[ptr+2] = _waves[j+2].Ainvk;
        (*constants)[ptr+3] = _waves[j+3].Ainvk;
        ptr += 4;
        // amplitude
        (*constants)[ptr+0] = _waves[j+0].A;
        (*constants)[ptr+1] = _waves[j+1].A;
        (*constants)[ptr+2] = _waves[j+2].A;
        (*constants)[ptr+3] = _waves[j+3].A;
        ptr += 4;
        // phase
        (*constants)[ptr+0] = _waves[j+0].phase;
        (*constants)[ptr+1] = _waves[j+1].phase;
        (*constants)[ptr+2] = _waves[j+2].phase;
        (*constants)[ptr+3] = _waves[j+3].phase;
        ptr += 4;
    }
}
