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

#include <osgbCollision/CollectVerticesVisitor.h>
#include <osgwTools/AbsoluteModelTransform.h>
#include <osg/Transform>
#include <osg/Geometry>
#include <osg/Geode>

#include <iostream>

using namespace osg;


namespace osgbCollision
{


CollectVerticesVisitor::CollectVerticesVisitor( osg::NodeVisitor::TraversalMode traversalMode )
    : osg::NodeVisitor( traversalMode )
{
    verts_ = new osg::Vec3Array;
    reset();
}

void CollectVerticesVisitor::reset()
{
    verts_->clear();
}

void CollectVerticesVisitor::apply( osg::Transform& node )
{
    // Override apply(Transform&) to avoid processing AMT nodes.
    const bool nonAMT = ( dynamic_cast< osgwTools::AbsoluteModelTransform* >( &node ) == NULL );
    if( nonAMT )
        _localNodePath.push_back( &node );

    traverse( node );

    if( nonAMT )
        _localNodePath.pop_back();
}

void CollectVerticesVisitor::apply( osg::Geode& geode )
{
    unsigned int idx;
    for( idx = 0; idx < geode.getNumDrawables(); idx++ )
        applyDrawable( geode.getDrawable( idx ) );
}

void CollectVerticesVisitor::applyDrawable( osg::Drawable* drawable )
{
    osg::Geometry* geom = drawable->asGeometry();
    if( geom == NULL )
        return;

    const osg::Vec3Array* in = dynamic_cast< const osg::Vec3Array* >( geom->getVertexArray() );
    if( in == NULL )
    {
        osg::notify( osg::WARN ) << "CollectVerticesVisitor: Non-Vec3Array vertex array encountered." << std::endl;
        return;
    }

    const osg::Matrix m = osg::computeLocalToWorld( _localNodePath );

    unsigned int idx;
    for( idx=0; idx < geom->getNumPrimitiveSets(); idx++ )
    {
        osg::PrimitiveSet* ps = geom->getPrimitiveSet( idx );
        unsigned int jdx;
        for( jdx=0; jdx < ps->getNumIndices(); jdx++ )
        {
            unsigned int index = ps->index( jdx );
            verts_->push_back( (*in)[ index ] * m );
        }
    }

    /*
    osg::Vec3Array::const_iterator iter;
    for( iter = in->begin(); iter != in->end(); iter++ )
    {
        verts_->push_back( *iter * m );
    }
    */
}


// osgbCollision
}
