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

#include <osgwTools/ForceFlattenTransforms.h>
#include <osg/Node>
#include <osg/Transform>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osg/Geometry>
#include <osgUtil/TransformAttributeFunctor>
#include <osg/Notify>
#include <string>


namespace osgwTools
{


ForceFlattenTransforms::ForceFlattenTransforms()
  : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
{}

void ForceFlattenTransforms::apply( osg::Transform& node )
{
    // This is a Transform that isn't a MatrixTransform and isn't a PAT.
    // If it is not an AMT, display a warning.
    if( node.className() != std::string( "AbsoluteModelTransform" ) )
        osg::notify( osg::INFO ) << "OSGToCollada: Warning: Non-MatrixTransform encountered: (" <<
            node.className() << ") " << node.getName() << std::endl;
    traverse( node );
}
void ForceFlattenTransforms::apply( osg::MatrixTransform& node )
{
    traverse( node );
    node.setMatrix( osg::Matrix::identity() );
}
void ForceFlattenTransforms::apply( osg::PositionAttitudeTransform& node )
{
    traverse( node );
    node.setPosition(osg::Vec3(0.0f,0.0f,0.0f));
    node.setAttitude(osg::Quat());
    node.setPivotPoint(osg::Vec3(0.0f,0.0f,0.0f));
    node.setScale(osg::Vec3(1.0f,1.0f,1.0f));
}

void ForceFlattenTransforms::apply( osg::Geode& node )
{
    osg::Matrix l2w = osg::computeLocalToWorld( getNodePath() );
    unsigned int idx;
    for( idx=0; idx<node.getNumDrawables(); idx++ )
    {
        osg::Drawable* draw( node.getDrawable( idx ) );

        osg::Geometry* geom( dynamic_cast< osg::Geometry* >( draw ) );
        if( geom )
        {
            // Requires 2.6.1, the necessary Geometry support didn't exist in 2.6.
            if( geom->containsSharedArrays() )
                geom->duplicateSharedArrays();
        }

        flattenDrawable( draw, l2w );
    }
}

void ForceFlattenTransforms::flattenDrawable( osg::Drawable* drawable, const osg::Matrix& matrix )
{
    if( drawable )
    {
        osgUtil::TransformAttributeFunctor tf(matrix);
        drawable->accept(tf);
        drawable->dirtyBound();
        drawable->dirtyDisplayList();
    }
}


// osgwTools
}
