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

#include <osgwTools/GeometryModifier.h>
#include <osgwTools/GeometryOperation.h>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PrimitiveSet>
#include <osgUtil/Optimizer>
#include <ostream>

namespace osgwTools {

GeometryModifier::GeometryModifier( const osg::NodeVisitor::TraversalMode mode )
  : osg::NodeVisitor( mode )
{
    reset();
}
GeometryModifier::GeometryModifier( GeometryOperation* geomOp )
  : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
    _geomOp( geomOp )
{
    reset();
}

void
GeometryModifier::reset()
{
    _drawableCount = _geometryCount = 0;
    _preVertices = _preIndices = _preTriangles = 0;
    _postVertices = _postIndices = _postTriangles = 0;
    _attemptDrawableMerge = 0;
}

GeometryModifier::~GeometryModifier()
{
}

void
GeometryModifier::apply( osg::Geode& geode )
{
    // merge drawables if possible for best results
    if (getDrawableMerge())
    {
        osgUtil::Optimizer::MergeGeometryVisitor mgv;
        mgv.setTargetMaximumNumberOfVertices(1000000);
        mgv.mergeGeode(geode);
    }

    for(unsigned int i=0;i<geode.getNumDrawables();++i)
    {
        _drawableCount++;
        osg::ref_ptr< osg::Geometry > geometry = geode.getDrawable(i)->asGeometry();
        if( geometry.valid() )
        {
            _geometryCount++;
            if( geometry->containsSharedArrays() )
                osg::notify( osg::DEBUG_INFO ) << "Warning! Geometry contains shared arrays" << std::endl;

            // Get statistics before
            incStatistics( geometry.get(), _preVertices, _preIndices, _preTriangles );

            osg::ref_ptr< osg::Geometry > newGeom = (*_geomOp)( *geometry );
            geode.replaceDrawable( geometry.get(), newGeom.get() );

            // Get statistics after
            incStatistics( newGeom.get(), _postVertices, _postIndices, _postTriangles );
        }
    }
}

void
GeometryModifier::incStatistics( const osg::Geometry* geom, unsigned int& vert, unsigned int& ind, unsigned int& tris )
{
    vert += geom->getVertexArray()->getNumElements();

    unsigned int idx;
    for( idx=0; idx < geom->getNumPrimitiveSets(); idx++ )
    {
        const osg::PrimitiveSet* ps( geom->getPrimitiveSet( idx ) );
        ind += ps->getNumIndices();

        switch( ps->getMode() )
        {
        case GL_TRIANGLES:
            tris += ps->getNumPrimitives();
            break;
        case GL_TRIANGLE_STRIP:
        case GL_QUAD_STRIP:
            tris += ps->getNumIndices() - 2;
            break;
        case GL_TRIANGLE_FAN:
        case GL_POLYGON:
            tris += ps->getNumIndices() - 1;
            break;
        case GL_QUADS:
            tris += ps->getNumPrimitives() * 2;
            break;
        default:
            break;
        }
    }
}

void
GeometryModifier::displayStatistics( std::ostream& ostr ) const
{
    ostr << "GeometryModifier statistics" << std::endl;
    ostr << "  GeometryOperation type: " << _geomOp->className() << std::endl;
    ostr << "  # Drawable: " << _drawableCount << ", # Geometry: " << _geometryCount << std::endl;
    ostr << "              Before\tAfter" << std::endl;
    ostr << "  Vertices:   " << _preVertices << "\t" << _postVertices << std::endl;
    ostr << "  Indices:    " << _preIndices << "\t" << _postIndices << std::endl;
    ostr << "  Triangles:  " << _preTriangles << "\t" << _postTriangles << std::endl;
}

}
