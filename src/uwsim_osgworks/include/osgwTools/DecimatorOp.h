/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgWorks is (C) Copyright 2009-2012 by Kenneth Mark Bryden
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License version 2.1 as published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *************** <auto-copyright.pl END do not edit this line> ***************/

#include <osgwTools/Export.h>
#include <osgwTools/GeometryOperation.h>
#include <osg/Geode>
#include <osg/Geometry>

namespace osgwTools {


/** \class DecimatorOp DecimatorOp.h <osgwTools/DecimatorOp.h>
\brief An osgUtil::Simplifier-like geometry reduction tool capable of removing edge vertices.

For more information, see \ref geomopt
*/
class OSGWTOOLS_EXPORT DecimatorOp : public GeometryOperation
{
public:
    DecimatorOp(double sampleRatio=1.0, double maximumError=FLT_MAX);
    DecimatorOp( const DecimatorOp& rhs, const osg::CopyOp& copyOp=osg::CopyOp::SHALLOW_COPY );

    META_Object(osgwTools,DecimatorOp);


    void setSampleRatio(float sampleRatio) { _sampleRatio = sampleRatio; }
    float getSampleRatio() const { return _sampleRatio; }

    void setMaximumError(float error) { _maximumError = error; }
    float getMaximumError() const { return _maximumError; }

    void setDoTriStrip(bool on) { _triStrip = on; }
    bool getDoTriStrip() const { return _triStrip; }

    void setSmoothing(bool on) { _smoothing = on; }
    bool getSmoothing() const { return _smoothing; }

    void setIgnoreBoundaries(bool setting){_ignoreBoundaries = setting;}
    bool getIgnoreBoundaries(){return _ignoreBoundaries;}

    void setMinPrimitives( unsigned int minPrim ) { _minPrim = minPrim; }
    unsigned int getMinPrimitives() const { return( _minPrim ); }

    /** \class ContinueDecimationCallback DecimatorOp.h <osgwTools/DecimatorOp.h>
    \brief Application control mechanism to continue or halt decimation.
    */
    class ContinueDecimationCallback : public osg::Referenced
        {
            public:
                /** Returns True if mesh is to continue being decimated. Returns False to stop decimation.*/
                virtual bool continueDecimation(const DecimatorOp& decimator, float nextError, unsigned int numOriginalPrimitives, unsigned int numRemainingPrimitives) const
                {
                    return decimator.continueDecimationImplementation(nextError, numOriginalPrimitives, numRemainingPrimitives);
                }
            
            protected:
                virtual ~ContinueDecimationCallback() {}
        };
     void setContinueDecimationCallback(ContinueDecimationCallback* cb) { _continueDecimationCallback = cb; }
     ContinueDecimationCallback* getContinueDecimationCallback() { return _continueDecimationCallback.get(); }
     const ContinueDecimationCallback* getContinueDecimationCallback() const { return _continueDecimationCallback.get(); }
        
        
     bool continueDecimation(float nextError, unsigned int numOriginalPrimitives, unsigned int numRemainingPrimitives) const
     {
         if (_continueDecimationCallback.valid()) return _continueDecimationCallback->continueDecimation(*this, nextError, numOriginalPrimitives, numRemainingPrimitives);
         else return continueDecimationImplementation(nextError, numOriginalPrimitives, numRemainingPrimitives);
     }
     virtual bool continueDecimationImplementation(float nextError, unsigned int numOriginalPrimitives, unsigned int numRemainingPrimitives) const
     {
         return ((float)numRemainingPrimitives > ((float)numOriginalPrimitives) * getSampleRatio()) && nextError<=getMaximumError();
        
     }

    virtual osg::Geometry* operator()( osg::Geometry& geom )
    {
        decimate( geom );

        return( &geom );
    }
    
     void decimate(osg::Geometry& geometry);

protected:
    
   double _sampleRatio;
   double _maximumError;
   bool  _triStrip;
   bool  _smoothing;
   bool  _ignoreBoundaries;
   unsigned int   _minPrim;
        
    osg::ref_ptr<ContinueDecimationCallback> _continueDecimationCallback;
};

}
