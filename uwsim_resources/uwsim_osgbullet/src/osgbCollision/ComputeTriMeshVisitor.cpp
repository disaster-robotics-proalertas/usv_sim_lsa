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

#include <osgbCollision/ComputeTriMeshVisitor.h>
#include <osg/Transform>
#include <osg/Drawable>
#include <osg/Geode>
#include <osg/PrimitiveSet>
#include <osg/TriangleFunctor>

#include <iostream>

using namespace osg;


namespace osgbCollision
{


/* \cond */
struct ComputeTriMeshFunc
{
    ComputeTriMeshFunc()
    {
        vertices = new osg::Vec3Array;

        vertices->clear();
    }

    void inline operator()( const osg::Vec3 v1, const osg::Vec3 v2, const osg::Vec3 v3, bool _temp )
    {
        vertices->push_back( v1 );
        vertices->push_back( v2 );
        vertices->push_back( v3 );
    }

    osg::ref_ptr< osg::Vec3Array > vertices;
};
/* \endcond */


ComputeTriMeshVisitor::ComputeTriMeshVisitor( osg::NodeVisitor::TraversalMode traversalMode )
    : osg::NodeVisitor( traversalMode )
{
    mesh = new osg::Vec3Array;
}

void ComputeTriMeshVisitor::reset()
{
    mesh->clear();
}

void ComputeTriMeshVisitor::apply( osg::Geode & geode )
{
    unsigned int idx;
    for( idx = 0; idx < geode.getNumDrawables(); idx++ )
        applyDrawable( geode.getDrawable( idx ) );
}

void ComputeTriMeshVisitor::applyDrawable( osg::Drawable * drawable )
{
    osg::TriangleFunctor< ComputeTriMeshFunc > functor;
    drawable->accept( functor );

    osg::Matrix m = osg::computeLocalToWorld( getNodePath() );
    osg::Vec3Array::iterator iter;
    for( iter = functor.vertices->begin(); iter != functor.vertices->end(); ++iter )
    {
        mesh->push_back( *iter * m );
    }
}


// osgbCollision
}
