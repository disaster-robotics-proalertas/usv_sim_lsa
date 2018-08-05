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

#ifndef __SHORT_EDGE_OP_H__
#define __SHORT_EDGE_OP_H__

#include <osgwTools/Export.h>
#include <osgwTools/GeometryOperation.h>
#include <osg/Geode>
#include <osg/Geometry>

#include <limits.h>


namespace osgwTools {


/** \class ShortEdgeOp ShortEdgeOp.h <osgwTools/ShortEdgeOp.h>
\brief A geometry reduction tool capable of removing edge vertices based on a maximum edge length expressed as a percentage of the smallest model bounding box dimension.

For more information, see \ref geomopt
*/
class OSGWTOOLS_EXPORT ShortEdgeOp : public GeometryOperation
{
public:

    ShortEdgeOp(double sampleRatio=1.0, double maxFeature=.1, unsigned int maxSteps=UINT_MAX);

    ShortEdgeOp( const ShortEdgeOp& rhs, const osg::CopyOp& copyOp=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgwTools,ShortEdgeOp);


    /** Percent of retained primitives. Default is 1.0 */
    void setSampleRatio( float sampleRatio ) { _sampleRatio = sampleRatio; }
    float getSampleRatio() const { return _sampleRatio; }

    /** Specify the maximum size of features/edges to consider for removal.
    This value is in the object space of the processed Geometry object.
    Edges with lengths less than this value are candidates for removal.
    Default is 0.1 */
    void setMaxFeature( float maxFeature ) { _maxFeature = maxFeature; }
    float getMaxFeature() const { return _maxFeature; }

    /** Optionally produce triangle strips as output. Default is false. */
    void setDoTriStrip(bool on) { _triStrip = on; }
    bool getDoTriStrip() const { return _triStrip; }

    /** Optionally run the SmoothingVisitor on the output tri mesh. Default is false. */
    void setSmoothing(bool on) { _smoothing = on; }
    bool getSmoothing() const { return _smoothing; }

    /** Controls boundary edge treatment. If false (default, boundary edges are
    considered for removal like any other edge. Setting to true prevents boundary
    edges from being removed. Default is false. */
    void setIgnoreBoundaries(bool setting){_ignoreBoundaries = setting;}
    bool getIgnoreBoundaries() const {return _ignoreBoundaries;}

    /** Minimum number of triangles that must be present in a Geometry Drawable in order for
    the short edge removal loop to continue. Default is 4. */
    void setMinPrimitives(unsigned int minPrim){_minPrim = minPrim;}
    unsigned int getMinPrimitives() const { return _minPrim;}

    /** Optionally limit the number of reduction iterations. Default is UINT_MAX (unlimited). */
    void setMaxSteps(unsigned int maxSteps){_maxSteps = maxSteps;}

    /** \brief Application control mechanism to continue or halt decimation.
    */
    class ContinueDecimationCallback : public osg::Referenced
        {
            public:
                /** Returns True if mesh is to continue being decimated. Returns False to stop decimation.*/
                virtual bool continueDecimation(const ShortEdgeOp& decimator, float nextError, unsigned int numOriginalPrimitives, unsigned int numRemainingPrimitives) const
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
         return ((float)numRemainingPrimitives > ((float)numOriginalPrimitives) * getSampleRatio()) && nextError<=getMaxFeature();
        
     }

    virtual osg::Geometry* operator()( osg::Geometry& geom )
    {
        decimate( geom );

        return( &geom );
    }
    
     void decimate(osg::Geometry& geometry);

protected:
    
    /** Minimum percent of primitives that will be retained. */
   double _sampleRatio;
    /** Edge length in model units, below which edges are candidates for removal. Default is 0.1 */
   double _maxFeature;
    /** if true, osgUtil::TriStripVisitor::stripify() is called after decimation and optional smoothing. */
   bool  _triStrip;
    /** if true, osgUtil::SmoothingVisitor::smooth() is called after decimation. */
   bool  _smoothing;
    /** if false, _IgnoreBoundaries prevents elimination of edges that are bounded by only one triangle. */
   bool  _ignoreBoundaries;
    /** Minimum number of primitives (triangles) that must be present in the drawable in order for recuction to proceed. */
   unsigned int   _minPrim;
    /** Maximum number of reduction steps to be executed subject to limitations of _sampleRatio, _maxFeature and _ignoreBoundaries. */
   unsigned int   _maxSteps;

    osg::ref_ptr<ContinueDecimationCallback> _continueDecimationCallback;
};

}

#endif

