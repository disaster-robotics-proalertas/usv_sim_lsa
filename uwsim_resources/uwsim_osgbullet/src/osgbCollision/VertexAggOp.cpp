/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgBullet is (C) Copyright 2009-2012 by Kenneth Mark Bryden
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

#include <osgbCollision/VertexAggOp.h>
#include <osgbCollision/CollisionShapes.h>
#include "BulletCollision/CollisionShapes/btShapeHull.h"
#include <btBulletCollisionCommon.h>
#include <osgbCollision/Utils.h>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/CopyOp>
#include <osg/io_utils>


namespace osgbCollision
{


typedef std::vector< osg::ref_ptr< Octree > > OctreeVec;
struct Octree : public osg::Referenced
{
    Octree() {}

    osg::BoundingBox _bb;
    osg::ref_ptr< osg::Vec3Array > _verts;

    OctreeVec _children;
};



VertexAggOp::VertexAggOp()
  : _maxVertsPerCell( 50 ),
    _minCellSize( osg::Vec3( 0., 0., 0. ) ),
    _useMinCellSize( false ),
    _createHull( true ),
    _psm( VertexAggOp::GEOMETRIC_MEAN )
{
}
VertexAggOp::VertexAggOp( const VertexAggOp& rhs, const osg::CopyOp& copyOp )
{
}
VertexAggOp::~VertexAggOp()
{
}

osg::Geometry*
VertexAggOp::operator()( osg::Geometry& geom )
{
    osg::Array* vArray = geom.getVertexArray();

    osg::Vec3Array* verts = dynamic_cast< osg::Vec3Array* >( vArray );
    if( verts != NULL )
    {
        osg::ref_ptr< Octree > oct = new Octree;
        {
            osg::BoundingBox& bb = oct->_bb;
            for( unsigned int jdx=0; jdx<verts->size(); jdx++ )
                bb.expandBy( (*verts)[ jdx ] );
        }
        oct->_verts = verts;
        recurseBuild( oct.get() );

        osg::Vec3Array* newV = new osg::Vec3Array;
        gatherVerts( oct.get(), newV );

        if( _createHull )
        {
            geom.setVertexArray( newV );
            createHull( geom );
        }
        else
        {
            geom.setVertexArray( newV );
            geom.removePrimitiveSet( 0, geom.getNumPrimitiveSets() );
            geom.addPrimitiveSet( new osg::DrawArrays( GL_POINTS, 0, newV->size() ) );
        }
    }

    return( &geom );
}

void
VertexAggOp::createHull( osg::Geometry& geom )
{
    osg::Vec3Array* oldV = dynamic_cast< osg::Vec3Array* >( geom.getVertexArray() );
    if( !oldV )
    {
        osg::notify( osg::ALWAYS ) << "VertexAggOp: Can't create convex hull." << std::endl;
        return;
    }
    btConvexHullShape* chs = new btConvexHullShape;
    osg::Vec3Array::const_iterator itr;
    for( itr = oldV->begin(); itr != oldV->end(); ++itr )
        chs->addPoint( osgbCollision::asBtVector3( *itr ) );

    osg::ref_ptr< osg::Node > n = osgbCollision::osgNodeFromBtCollisionShape( chs );
    osg::Geode* newGeode = dynamic_cast< osg::Geode* >( n.get() );
    
    if( newGeode == NULL )
    {
        osg::notify( osg::FATAL ) << "Got NULL geode from osgNodeFromBtCollisionShape" << std::endl;
        return;
    }
    
    osg::Drawable* newDraw = newGeode->getDrawable( 0 );
    osg::Geometry* newGeom = dynamic_cast< osg::Geometry* >( newDraw );
    
    if( newGeom == NULL )
    {
        osg::notify( osg::FATAL ) << "Got NULL geometry from osgNodeFromBtCollisionShape" << std::endl;
        return;
    }

    geom.setVertexArray( newGeom->getVertexArray() );
    geom.setColorArray( newGeom->getColorArray() );
    geom.setColorBinding( newGeom->getColorBinding() );
    geom.removePrimitiveSet( 0, geom.getNumPrimitiveSets() );
    geom.addPrimitiveSet( newGeom->getPrimitiveSet( 0 ) );
}

void
VertexAggOp::recurseBuild( Octree* cell ) const
{
    osg::Vec3Array* verts = cell->_verts.get();
    if( verts->size() <= _maxVertsPerCell )
        return;

    const osg::Vec3 center( cell->_bb.center() );
    const osg::Vec3 cellMax( cell->_bb._max );
    const unsigned int posX( 1<<0 );
    const unsigned int posY( 1<<1 );
    const unsigned int posZ( 1<<2 );

    cell->_children.resize( 8 );
    unsigned int idx;
    for( idx=0; idx<8; idx++ )
    {
        Octree* oct = new Octree;
        cell->_children[ idx ] = oct;

        osg::Vec3 cMin = cell->_bb._min;
        osg::Vec3 cMax = center;
        if( idx & posX )
        {
            cMin.x() = center.x();
            cMax.x() = cellMax.x();
        }
        if( idx & posY )
        {
            cMin.y() = center.y();
            cMax.y() = cellMax.y();
        }
        if( idx & posZ )
        {
            cMin.z() = center.z();
            cMax.z() = cellMax.z();
        }
        oct->_bb.set( cMin, cMax );
        oct->_verts = new osg::Vec3Array;
    }
    for( idx=0; idx<verts->size(); idx++ )
    {
        osg::Vec3& v = (*verts)[ idx ];
        unsigned int childIdx( 0 );
        if( v.x() > center.x() )
            childIdx |= posX;
        if( v.y() > center.y() )
            childIdx |= posY;
        if( v.z() > center.z() )
            childIdx |= posZ;
        cell->_children[ childIdx ]->_verts->push_back( v );
    }

    verts->clear();

    for( idx=0; idx<8; idx++ )
        recurseBuild( cell->_children[ idx ].get() );
}

void
VertexAggOp::gatherVerts( Octree* cell, osg::Vec3Array* verts ) const
{
    if( cell->_verts->size() > 0 )
        verts->push_back( representativeVert( cell->_verts.get() ) );
    else if( cell->_children.size() > 0 )
    {
        int idx;
        for( idx=0; idx<8; idx++ )
        {
            Octree* child = cell->_children[ idx ].get();
            if( child != NULL )
                gatherVerts( child, verts );
        }
    }
}

osg::Vec3
VertexAggOp::representativeVert( osg::Vec3Array* verts ) const
{
    osg::Vec3 rep( 0., 0., 0. );

    if( _psm == GEOMETRIC_MEAN )
    {
        unsigned int idx;
        for( idx=0; idx<verts->size(); idx++ )
            rep += (*verts)[ idx ];
        rep /= verts->size();
    }
    else if( _psm == BOUNDING_BOX_CENTER )
    {
        osg::BoundingBox bb;
        unsigned int idx;
        for( idx = 0; idx < verts->size(); idx++ )
            bb.expandBy( (*verts)[ idx ] );
        rep = bb.center();
    }

    return( rep );
}


// osgbCollision
}
