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

#include "osgwTools/Transform.h"
#include <osg/BoundingSphere>
#include <osg/BoundingBox>
#include <osg/Matrix>


namespace osgwTools {


osg::BoundingSphere transform( const osg::Matrix& m, const osg::BoundingSphere& sphere )
{
    osg::BoundingSphere::vec_type xdash = sphere._center;
    xdash.x() += sphere._radius;
    xdash = xdash * m;

    osg::BoundingSphere::vec_type ydash = sphere._center;
    ydash.y() += sphere._radius;
    ydash = ydash * m;

    osg::BoundingSphere::vec_type zdash = sphere._center;
    zdash.z() += sphere._radius;
    zdash = zdash * m;

    osg::BoundingSphere newSphere;
    newSphere._center = sphere._center * m;

    xdash -= newSphere._center;
    osg::BoundingSphere::value_type len_xdash = xdash.length();

    ydash -= newSphere._center;
    osg::BoundingSphere::value_type len_ydash = ydash.length();

    zdash -= newSphere._center;
    osg::BoundingSphere::value_type len_zdash = zdash.length();

    newSphere._radius = len_xdash;
    if( newSphere._radius < len_ydash )
        newSphere._radius = len_ydash;
    if( newSphere._radius < len_zdash )
        newSphere._radius = len_zdash;

    return( newSphere );
}

osg::BoundingBox transform( const osg::Matrix& m, const osg::BoundingBox& box )
{
    osg::ref_ptr< osg::Vec3Array > v = new osg::Vec3Array;
    v->resize( 8 );
    (*v)[0].set( box._min );
    (*v)[1].set( box._max.x(), box._min.y(), box._min.z() );
    (*v)[2].set( box._max.x(), box._min.y(), box._max.z() );
    (*v)[3].set( box._min.x(), box._min.y(), box._max.z() );
    (*v)[4].set( box._max );
    (*v)[5].set( box._min.x(), box._max.y(), box._max.z() );
    (*v)[6].set( box._min.x(), box._max.y(), box._min.z() );
    (*v)[7].set( box._max.x(), box._max.y(), box._min.z() );

    transform( m, v.get() );

    osg::BoundingBox newBox( (*v)[0], (*v)[1] );
    newBox.expandBy( (*v)[2] );
    newBox.expandBy( (*v)[3] );
    newBox.expandBy( (*v)[4] );
    newBox.expandBy( (*v)[5] );
    newBox.expandBy( (*v)[6] );
    newBox.expandBy( (*v)[7] );

    return( newBox );
}

void transform( const osg::Matrix& m, osg::Vec3Array* verts, bool normalize )
{
    osg::Vec3Array::iterator itr;
    for( itr = verts->begin(); itr != verts->end(); itr++ )
    {
        *itr = *itr * m;
        if( normalize )
            itr->normalize();
    }
    verts->dirty();
}

void transform( const osg::Matrix& m, osg::Geometry* geom )
{
    if( geom == NULL )
        return;

    // Transform the vertices by the specified matrix.
    osg::Vec3Array* v = dynamic_cast< osg::Vec3Array* >( geom->getVertexArray() );
    if( v != NULL )
    {
        transform( m, v );
    }

    // Transform the normals by the upper-left 3x3 matrix (no translation)
    // and rescale the normals in case the matrix contains scalng.
    osg::Vec3Array* n = dynamic_cast< osg::Vec3Array* >( geom->getNormalArray() );
    if( n != NULL )
    {
        osg::Matrix m3x3( m );
        m3x3.setTrans( 0., 0., 0. );
        transform( m3x3, n, true );
    }

    geom->dirtyDisplayList();
    geom->dirtyBound();
}

void transform( const osg::Matrix& m, osg::Geode* geode )
{
    if( geode == NULL )
        return;

    unsigned int idx;
    for( idx=0; idx<geode->getNumDrawables(); idx++ )
    {
        osg::Geometry* geom = geode->getDrawable( idx )->asGeometry();
        if( geom != NULL )
        {
            transform( m, geom );
        }
        else
        {
            // TBD need support for OSG shape drawables.
            osg::notify( osg::WARN ) << "osgwTools::transform can't transform non-Geometry yet." << std::endl;
        }
    }
}


// osgwTools
}
