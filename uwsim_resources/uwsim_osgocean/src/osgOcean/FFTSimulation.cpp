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
#include <osgOcean/FFTSimulation>
#include <osgOcean/RandUtils>

#include <complex>
#include <vector>

using namespace osgOcean;

// The FFTW and FFTSS docs advise to use fftw_malloc and fftw_free instead
// of new and delete for their data, as they claim to make special guarantees
// of data alignment that the default new cannot do. In practice, they do not 
// seem to make a significant difference in performance, but I guess we'd
// better err on the side of caution and follow their recommendations.
#define USE_FFTW_MALLOC

// Sanity check - one and only one of the 3 USE_ flags should be defined.
#if (defined(USE_FFTW3) && defined(USE_FFTW3F)) || \
    (defined(USE_FFTW3) && defined(USE_FFTSS))  || \
    (defined(USE_FFTW3F) && defined(USE_FFTSS)) || \
    (!defined(USE_FFTW3) && !defined(USE_FFTW3F) && !defined(USE_FFTSS))
#error Must use one of FFTW3 (double-precision), FFTW3F (single-precision) or FFTSS!
#endif

#if defined(USE_FFTW3) || defined(USE_FFTW3F)

  #include <fftw3.h>

  #if defined(USE_FFTW3)              // double precision
    typedef double fftw_data_type;
  #else //if defined(USE_FFTW3F)      // single precision
    typedef float fftw_data_type;

    // alias the functions so that we can use the same function calls in both
    // the double- and single-precision FFTW paths.
    #define fftw_complex         fftwf_complex
    #define fftw_plan            fftwf_plan
    #define fftw_destroy_plan    fftwf_destroy_plan
    #define fftw_plan_dft_2d     fftwf_plan_dft_2d
    #define fftw_execute         fftwf_execute
    #define fftw_malloc          fftwf_malloc
    #define fftw_free            fftwf_free
  #endif

#elif defined(USE_FFTSS)

  #include <fftw3compat.h>
  typedef double fftw_data_type;      // FFTSS is double-precision only

#endif

typedef std::complex<fftw_data_type> complex;

class FFTSimulation::Implementation
{
private:
    const double _PI2;             /**< 2*PI */
    const double _GRAVITY;         /**< Gravitational constant 9.81 */
    const double _GRAVITY2;        /**< Gravitational constant squared */

    int _N;                        /**< Size of FFT grid 2^n ie 128,64,32 etc. */
    int _numPoints;                /**< Size of FFT squared */
    int _nOver2;                   /**< Half fourier size (_N/2)*/
    osg::Vec2f _windDir;           /**< Direction of wind. */
    float _windSpeed4;             /**< Wind speed (m/s) to power 4 */
    float _A;                      /**< Wave scale modifier. */
    float _length;                 /**< Real world tile resolution (m). */
    float _w0;                     /**< Base frequency (2PI / looptime). */
    float _maxWave;                /**< Maximum wave size for current wind speed */
    float _depth;                  /**< Depth (m) */
    float _reflDampFactor;         /**< Dampen reflections going against the wind */

    fftw_complex *_complexData0;   /**< 2D complex data array used for FFT input*/
    fftw_complex *_complexData1;   /**< 2D complex data array used for FFT input */

    fftw_complex *_realData0;      /**< 2D complex data array used for FFT output */
    fftw_complex *_realData1;      /**< 2D complex data array used for FFT output  */

    fftw_plan _fftPlan0;           /**< 2D Inverse FFT plan */
    fftw_plan _fftPlan1;           /**< 2D Inverse FFT plan */

    std::vector< complex > _baseAmplitudes; /**< Base fourier amplitudes */
    std::vector< complex > _curAmplitudes;  /**< Current fourier amplitudes */  

    std::vector< complex > _h0TildeK;
    std::vector< complex > _h0TildeKconj;
    std::vector< float > _wK;
    std::vector< osg::Vec2 > _Kh;

public:
    /** Constructor.
    * Provides default parameters for a calm ocean surface.
    * Computes base amplitudes and initialises FFT plans and arrays.
    * @param fourierSize Size of FFT grid 2^n ie 128,64,32 etc.
    * @param windDir Direction of wind.
    * @param windSpeed Speed of wind (m/s).
    * @param waveScale Wave height modifier.
    * @param loopTime Time for animation to repeat (secs).
    */
    Implementation(
        int fourierSize = 64,
        const osg::Vec2f& windDir = osg::Vec2f(1.0f, 1.0f),
        float windSpeed  = 12.f,
        float depth = 1000.f,
        float reflectionDamping = 0.35f,
        float waveScale = 1e-9,    
        float tileRes = 256.f,
        float loopTime  = 10.f
        );

    /** Destructor.
    * Cleans up FFT plans and arrays.
    */
    ~Implementation(void);

    /** Set the current time and computes the current fourier amplitudes */
    void setTime(float time);    

    /** Compute the current height field. 
    * Executes an FFT transform to convert the current fourier amplitudes to real height values.
    * @param heights must be created before passing in. Function will resize and overwrite the contents with current data.
    */
    void computeHeights( osg::FloatArray* heights ) const;

    /** Compute the (x,y) displacements for choppy wave effect
    * Executes two FFT transforms to convert the current fourier amplitudes to real x,y displacements
    * @param scaleFactor defines the magnitude of the displacements. Typically a negative value ( -3.0 > val < -1.0 ).
    * @param waveDisplacements must be created before passing in. Function will resize and overwrite the contents with the computed displacements.
    */
    void computeDisplacements( const float& scaleFactor, osg::Vec2Array* waveDisplacements ) const;

private:
    float phillipsSpectrum(const osg::Vec2f& K) const;

    /** Computes the base fourier amplitudes htilde0.*/
    void computeBaseAmplitudes();

    /** Computes the current fourier amplitudes htilde.*/
    inline void computeCurrentAmplitudes(float time);

    void computeConstants( void );
};

FFTSimulation::Implementation::Implementation( int fourierSize,
                                               const osg::Vec2f& windDir,
                                               float windSpeed,
                                               float depth,
                                               float reflectionDamping,
                                               float waveScale,
                                               float tileRes,
                                               float loopTime ):
    _PI2            ( 2.0*osg::PI ),
    _GRAVITY        ( 9.81 ),
    _GRAVITY2       ( 96.2361 ),
    _N              ( fourierSize ), 
    _numPoints      ( _N*_N ),
    _nOver2         ( fourierSize/2 ),
    _windDir        ( windDir ), 
    _windSpeed4     ( windSpeed*windSpeed*windSpeed*windSpeed ), 
    _A              ( float(_N)*waveScale ),
    _length         ( tileRes ),
    _w0             ( _PI2 / loopTime ),
    _maxWave        ( _windSpeed4/_GRAVITY2 ),
    _depth          ( depth ),
    _reflDampFactor ( reflectionDamping )
{
    _curAmplitudes.resize( _numPoints );
    computeBaseAmplitudes();
    computeConstants();

#ifdef USE_FFTW_MALLOC
    _complexData0 = (fftw_complex*)fftw_malloc(_numPoints * sizeof(fftw_complex));
    _complexData1 = (fftw_complex*)fftw_malloc(_numPoints * sizeof(fftw_complex));

    _realData0 = (fftw_complex*)fftw_malloc(_numPoints * sizeof(fftw_complex));
    _realData1 = (fftw_complex*)fftw_malloc(_numPoints * sizeof(fftw_complex));
#else
    _complexData0 = new fftw_complex[ _numPoints ];
    _complexData1 = new fftw_complex[ _numPoints ];

    _realData0 = new fftw_complex[ _numPoints ];
    _realData1 = new fftw_complex[ _numPoints ];
#endif

    _fftPlan0 = fftw_plan_dft_2d( _N, _N, _complexData0, _realData0, FFTW_BACKWARD, FFTW_ESTIMATE );
    _fftPlan1 = fftw_plan_dft_2d( _N, _N, _complexData1, _realData1, FFTW_BACKWARD, FFTW_ESTIMATE );
}

FFTSimulation::Implementation::~Implementation()
{
    fftw_destroy_plan(_fftPlan0);
    fftw_destroy_plan(_fftPlan1);

#ifdef USE_FFTW_MALLOC
    fftw_free(_complexData0);
    fftw_free(_complexData1);

    fftw_free(_realData0);
    fftw_free(_realData1);
#else
    delete[] _complexData0;
    delete[] _complexData1;

    delete[] _realData0;
    delete[] _realData1;
#endif
}

float FFTSimulation::Implementation::phillipsSpectrum(const osg::Vec2f& K) const
{
    float k2 = K.length2();

    if (k2 == 0.f) 
        return 0.f;

    float k4 = k2 * k2;

    float KdotW = K*_windDir;

    float KdotWhat = KdotW*KdotW/k2;

    float eterm = exp( -_GRAVITY2 / (k2*_windSpeed4) ) / k4;

    const float damping = 0.000001f;

    float specResult = _A * eterm * KdotWhat * exp( -k2 * _maxWave * damping );    

    if (KdotW < 0.f)    
        specResult *= _reflDampFactor;

    return specResult;
}

void FFTSimulation::Implementation::computeBaseAmplitudes()
{
    _baseAmplitudes.resize( (_N+1)*(_N+1) );

    osg::Vec2 K;
    float oneOverLen = 1.f / _length;
    float real,imag;

    for (int y = 0, y2 = -_nOver2; y <= _N; ++y, ++y2) 
    {
        K.y() = _PI2*y2*oneOverLen;

        for (int x = 0, x2 = -_nOver2; x <= _N; ++x, ++x2) 
        {
            K.x() = _PI2*x2*oneOverLen;

            RandUtils::gaussianRand(real,imag);

#if defined(USE_FFTW3F)
            _baseAmplitudes[y*(_N+1)+x] = complex(real,imag) * sqrtf( 0.5f * phillipsSpectrum(K) );
#else
            _baseAmplitudes[y*(_N+1)+x] = complex(real,imag) * sqrt( 0.5 * (double)phillipsSpectrum(K) );
#endif
        }
    }
}

void FFTSimulation::Implementation::computeConstants( void )
{
    float oneOverLen = 1.f/(float)_length;

    _h0TildeK.resize(_numPoints);
    _h0TildeKconj.resize(_numPoints);
    _wK.resize(_numPoints);
    _Kh.resize(_numPoints);

    int ptr = 0;

    osg::Vec2 K;
    osg::Vec2 Kh0(0,0);
    
    float klen = 0.f;
    float wK  = 0.f;

    for(int y = 0; y < _N; ++y )
    {
        K.y() = _PI2 * ( (float)(y-_nOver2) * oneOverLen );

        for( int x = 0; x < _N; ++x )
        {
            K.x() = _PI2 * ( (float)(x-_nOver2) * oneOverLen );

            ptr = y*_N+x;

            _h0TildeK[ptr] = _baseAmplitudes[ y*(_N+1)+x ];
            _h0TildeKconj[ptr] = conj( _baseAmplitudes[ (_N-y)*(_N+1)+(_N-x) ] );

            klen = K.length();

            wK = sqrt( _GRAVITY * klen * tanh(klen*_depth) );
            _wK[ptr] = floor(wK/_w0)*_w0;

            if (klen != 0)
                _Kh[ptr] = K * (1.f/klen);
            else
                _Kh[ptr] = Kh0;
        }
    }
}

void FFTSimulation::Implementation::computeCurrentAmplitudes(float time)
{
    for (int y = 0; y < _N; ++y) 
    {
        for (int x = 0; x < _N; ++x) 
        {
            int ptr = y*_N+x;

            float wT = _wK[ptr] * time;
            float cwT = cos(wT);
            float swT = sin(wT);

            _curAmplitudes[y*_N+x] = _h0TildeK[ptr] * complex(cwT, swT) + _h0TildeKconj[ptr] * complex(cwT, -swT);
        }
    }
}

void FFTSimulation::Implementation::setTime(float time)
{
    computeCurrentAmplitudes(time);
}

void FFTSimulation::Implementation::computeHeights( osg::FloatArray* waveheights ) const
{
    // populate input array
    for(int y = 0; y < _N; ++y )
    {
        for (int x = 0; x < _N; ++x) 
        {
            int ptr = y*_N+x;

            _complexData0[ptr][0] = _curAmplitudes[ptr].real();
            _complexData0[ptr][1] = _curAmplitudes[ptr].imag();
        }
    }

    fftw_execute(_fftPlan0);

    if (waveheights->size() != (unsigned int)(_numPoints) ){
        waveheights->resize(_numPoints);
    }

    const float signs[2] = { 1.f, -1.f };

    for(int y = 0; y < _N; ++y)
    {
        for(int x = 0; x < _N; ++x )
        {
            waveheights->at(y*_N+x) = _realData0[x*_N+y][0]  * signs[(x + y) & 1];
        }
    }
}

void FFTSimulation::Implementation::computeDisplacements(const float& scaleFactor, 
                                                         osg::Vec2Array* waveDisplacements) const
{
    for (int y = 0; y < _N; ++y) 
    {
        for (int x = 0; x < _N; ++x) 
        {
            int ptr = y*_N+x;
            int flipPtr = x*_N+y;

            const complex& c = _curAmplitudes[ptr];

            _complexData0[flipPtr][0] =  c.imag() * _Kh[ptr].x();
            _complexData0[flipPtr][1] = -c.real() * _Kh[ptr].x();

            _complexData1[flipPtr][0] =  c.imag() * _Kh[ptr].y();
            _complexData1[flipPtr][1] = -c.real() * _Kh[ptr].y();
        }
    }

    fftw_execute(_fftPlan0);
    fftw_execute(_fftPlan1);

    if (waveDisplacements->size() != (unsigned int)(_numPoints) )
        waveDisplacements->resize(_numPoints);

    const float signs[2] = { 1.f, -1.f };

    unsigned int ptr = 0;

    osg::Vec2 real;

    for (int y = 0; y < _N; ++y)
    {
        for (int x = 0; x < _N; ++x) 
        {
            ptr = x*_N+y;

            double s = signs[(x + y) & 1];
            real.x() = _realData0[ptr][0];
            real.y() = _realData1[ptr][0];
            waveDisplacements->at(y*_N+x) = real * s * (double)scaleFactor;
        }
    }
}


FFTSimulation::FFTSimulation( int fourierSize,
                              const osg::Vec2f& windDir,
                              float windSpeed,
                              float depth,
                              float reflectionDamping,
                              float waveScale,
                              float tileRes,
                              float loopTime)
    : _implementation( new Implementation(fourierSize, windDir, windSpeed, depth, reflectionDamping, waveScale, tileRes, loopTime) )
{
}

FFTSimulation::~FFTSimulation()
{
    delete _implementation;
}

void FFTSimulation::setTime(float time)
{
    _implementation->setTime(time);
}

void FFTSimulation::computeHeights( osg::FloatArray* heights ) const
{
    _implementation->computeHeights(heights);
}

void FFTSimulation::computeDisplacements( const float& scaleFactor, osg::Vec2Array* waveDisplacements ) const
{
    _implementation->computeDisplacements(scaleFactor, waveDisplacements);
}
