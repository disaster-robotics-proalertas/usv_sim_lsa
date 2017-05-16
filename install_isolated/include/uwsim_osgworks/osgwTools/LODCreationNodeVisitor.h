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

#ifndef __OSGWTOOLS_LOD_CREATION_NODE_VISITOR_H__
#define __OSGWTOOLS_LOD_CREATION_NODE_VISITOR_H__

#include <osgwTools/Export.h>
#include <osg/NodeVisitor>
#include <osg/Geode>
#include <osg/Geometry>

#include <iterator>

namespace osgwTools {

typedef std::set<osg::Geode*> GeodeSet;
typedef std::pair<double, double> LODPair;
typedef std::vector< LODPair > LODPairList;
typedef std::vector< osg::ref_ptr<osg::LOD> > LODList;

/** \class GeodeReducableCallback LODCreationNodeVisitor.h <osgwTools/LODCreationNodeVisitor.h>
\brief Callback to decide which Geodes to store */
class OSGWTOOLS_EXPORT GeodeReducableCallback : public osg::Referenced
{
public:
    /** Returns true if Geode should be added to set for later treatment
	Override to implement the desired behavior. */
	virtual bool testGeodeReducable( osg::Geode *geode, unsigned int minVertices, unsigned int minPrimitives ) const
    {
        return false;
    } // testGeodeReducable

    virtual ~GeodeReducableCallback() {}
}; // GeodeReducableCallback



/** \class BasicGeodeReducableCallback LODCreationNodeVisitor.h <osgwTools/LODCreationNodeVisitor.h>
\brief Callback to select highest LOD child */
class OSGWTOOLS_EXPORT BasicGeodeReducableCallback : public osgwTools::GeodeReducableCallback
{
public:
    /** returns true if Geode should be added to set for later treatment */
	bool testGeodeReducable( osg::Geode *geode, unsigned int minVertices, unsigned int minPrimitives ) const
    {
        unsigned int drawableCount = 0;
        unsigned int geometryCount = 0;
        unsigned int numVertices = 0;
        unsigned int numPrimitives = 0;

        for(unsigned int i = 0; i < geode->getNumDrawables(); ++i)
        {
            ++drawableCount;
            osg::ref_ptr< osg::Geometry > geometry = geode->getDrawable(i)->asGeometry();
            if( geometry.valid() )
            {
                ++geometryCount;
                if( geometry->containsSharedArrays() )
                    osg::notify( osg::ALWAYS ) << "Warning! Geometry contains shared arrays" << std::endl;

                // Get statistics
                numVertices += geometry->getVertexArray()->getNumElements();

                unsigned int idx;
                for( idx = 0; idx < geometry->getNumPrimitiveSets(); ++idx )
                {
                    const osg::PrimitiveSet* ps( geometry->getPrimitiveSet( idx ) );
                    numPrimitives += ps->getNumPrimitives();
                }

            }
        }

        return (numVertices > minVertices || numPrimitives > minPrimitives);
    } // testGeodeReducable
}; // BasicGeodeReducableCallback


/** \class LODCreationNodeVisitor LODCreationNodeVisitor.h <osgwTools/LODCreationNodeVisitor.h>
\brief A node visitor for creating LOD nodes to replace Geodes with simplified geometry
at user-supplied levels of detail and pixel size constraints.

*/
class OSGWTOOLS_EXPORT LODCreationNodeVisitor : public osg::NodeVisitor
{
public:
    LODCreationNodeVisitor( GeodeReducableCallback* cb=NULL );
    virtual ~LODCreationNodeVisitor() {}

    void setLODPairs( LODPairList& lodPairList )
    {
        _lodPairList.clear();
        // copy the pairs
        std::copy( lodPairList.begin(), lodPairList.end(), std::inserter( _lodPairList, _lodPairList.begin() ) );
    }
    /** Minimum vertices in order to proceed with LOD reduction. Default is 100. */
    void setTestMinVertices( unsigned int minVertices ) { _minTestVertices = minVertices; }
    /** Minimum primitives in order to proceed with LOD reduction. Default is 100. */
    void setTestMinPrimitives( unsigned int minPrimitives ) { _minTestPrimitives = minPrimitives;}
    /** Minimum percent of retained primitives. Default is 0.01f. */
    void setMinRetentionPercent( float maxDec ) { _minRetentionPercent = maxDec; }
    /** Controls boundary edge treatment. If false (default, boundary edges are
    considered for removal like any other edge. Setting to true prevents boundary
    edges from being removed. Default is false. */
    void setIgnoreBoundaries( bool ignore ) { _decIgnoreBoundaries = ignore; }
    /** Attempt to merge drawables for simultaneous processing. Default is false. */
    void setAttemptMerge( bool attempt ) { _attemptMerge = attempt; }
    /** Attempts to create triangle strips after decimation. Default is false. */
    void setDoTriStrip(bool on) { _triStrip = on; }
    /** Smoothing is run after decimation. Default is false. */
    void setSmoothing(bool on) { _smoothing = on; }

    GeodeSet& getLODCandidates( void ) { return (_lodCandidates); }
    LODPairList& getLODPairs( void ) { return (_lodPairList); }

	void apply( osg::Node& node );

    /** Calling code must call this function to create LODs.
    This NodeVisitor collects Geodes, but doesn't actually process
    them until the calling code executes this member function. */
	unsigned int finishProcessingGeodes( void );

protected:
	void processNode( osg::Node& node );

    GeodeSet _lodCandidates;
    /** _lodPairList Calling code can supply a pair list of rely on defaults
    Order of items in pairs is 1) smallest pixel size for LOD to be used, 
    2) maximum feature size to be removed as % of node bounding sphere diameter. */
    LODPairList _lodPairList;
    /** if false, _decIgnoreBoundaries prevents elimination of edges that are bounded by only one triangle. */
    bool _decIgnoreBoundaries;
    /** if true, osgUtil::SmoothingVisitor::smooth() is called after decimation. */
    bool _smoothing;
    /** if true, osgUtil::TriStripVisitor::stripify() is called after decimation and optional smoothing. */
    bool _triStrip;
    /** if set, _attemptMerge attempts to merge drawables in model file prior to executing lod reduction. 
    Subject to limitations of osgUtil::Optimizer::MergeGeometryVisitor. */
    bool _attemptMerge;
    /** _geodesLocated, _geodesProcessed for internal control. */
	unsigned int _geodesLocated, _geodesProcessed;
    /** _minTestVertices, _minTestPrimitives can be supplied to control which Geodes will be selected for
    decimation using the default callback function. */
    unsigned int _minTestVertices, _minTestPrimitives;
    /** Minimum percent of primitives that will be retained. */
    float _minRetentionPercent;

    /** Calling code can supply a callback function to decide which Geodes should be decimated or rely on
    default method based on _minTestVertices and _minTestPrimitives. */
    osg::ref_ptr<GeodeReducableCallback> _geodeReducableCallback;
};


// osgwTools
}


// __OSGWTOOLS_LOD_CREATION_NODE_VISITOR_H__
#endif
