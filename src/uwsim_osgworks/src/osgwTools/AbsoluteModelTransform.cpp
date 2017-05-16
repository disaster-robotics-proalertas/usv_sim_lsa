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

#include <osgwTools/AbsoluteModelTransform.h>
#include <osgUtil/CullVisitor>
#include <osg/NodeVisitor>
#include <osg/Transform>
#include <osg/Matrix>

#include <string>
#include <osg/io_utils>


namespace osgwTools
{


AbsoluteModelTransform::AbsoluteModelTransform()
{
    setReferenceFrame( osg::Transform::ABSOLUTE_RF );
}
AbsoluteModelTransform::AbsoluteModelTransform( const osg::Matrix& m )
  : _matrix( m )
{
    setReferenceFrame( osg::Transform::ABSOLUTE_RF );
}
AbsoluteModelTransform::AbsoluteModelTransform( const AbsoluteModelTransform& rhs, const osg::CopyOp& copyop )
  : osg::Transform( rhs, copyop ),
    _matrix( rhs._matrix )
{
    setReferenceFrame( osg::Transform::ABSOLUTE_RF );
}
AbsoluteModelTransform::~AbsoluteModelTransform()
{
}


bool
AbsoluteModelTransform::computeLocalToWorldMatrix( osg::Matrix& matrix, osg::NodeVisitor* nv ) const
{
    if( getReferenceFrame() == osg::Transform::ABSOLUTE_RF )
    {
        osg::Matrix view;
        if( !nv )
            osg::notify( osg::INFO ) << "AbsoluteModelTransform: NULL NodeVisitor; can't get view." << std::endl;
        else if( nv->getVisitorType() != osg::NodeVisitor::CULL_VISITOR )
            osg::notify( osg::INFO ) << "AbsoluteModelTransform: NodeVisitor is not CullVisitor; can't get view." << std::endl;
        else
        {
            osgUtil::CullVisitor* cv = dynamic_cast< osgUtil::CullVisitor* >( nv );
#ifdef SCENEVIEW_ANAGLYPHIC_STEREO_SUPPORT
            // If OSG_STEREO=ON is in the environment, SceneView hides the view matrix
            // in a stack rather than placing it in a Camera node. Enable this code
            // (using CMake) to use a less-efficient way to compute the view matrix that
            // is compatible with SceneView's usage.
            osg::NodePath np = nv->getNodePath();
            np.pop_back();
            osg::Matrix l2w = osg::computeLocalToWorld( np );
            osg::Matrix invL2w = osg::Matrix::inverse( l2w );
            view = invL2w * *( cv->getModelViewMatrix() );
#else
            // Faster way to determine the view matrix, but not compatible with
            // SceneView anaglyphic stereo.
            osg::Camera* cam = cv->getCurrentCamera();
            cam->computeLocalToWorldMatrix( view, cv );
#endif
        }
        matrix = ( _matrix * view );
    }
    else
        // RELATIVE_RF
        matrix.preMult(_matrix);

    return( true );
}

bool
AbsoluteModelTransform::computeWorldToLocalMatrix( osg::Matrix& matrix, osg::NodeVisitor* nv ) const
{
    osg::Matrix inv( osg::Matrix::inverse( _matrix ) );
    if( getReferenceFrame() == osg::Transform::ABSOLUTE_RF )
    {
        osg::Matrix invView;
        if( !nv )
            osg::notify( osg::NOTICE ) << "AbsoluteModelTransform: NULL NodeVisitor; can't get invView." << std::endl;
        else if( nv->getVisitorType() != osg::NodeVisitor::CULL_VISITOR )
            osg::notify( osg::NOTICE ) << "AbsoluteModelTransform: NodeVisitor is not CullVisitor; can't get invView." << std::endl;
        else
        {
            osgUtil::CullVisitor* cv = dynamic_cast< osgUtil::CullVisitor* >( nv );
#ifdef SCENEVIEW_ANAGLYPHIC_STEREO_SUPPORT
            // If OSG_STEREO=ON is in the environment, SceneView hides the view matrix
            // in a stack rather than placing it in a Camera node. Enable this code
            // (using CMake) to use a less-efficient way to compute the view matrix that
            // is compatible with SceneView's usage.
            osg::NodePath np = nv->getNodePath();
            np.pop_back();
            osg::Matrix l2w = osg::computeLocalToWorld( np );
            invView = *( cv->getModelViewMatrix() ) * l2w;
#else
            osg::Camera* cam = cv->getCurrentCamera();
            cam->computeWorldToLocalMatrix( invView, cv );
#endif
        }
        matrix = ( invView * inv );
    }
    else
        // RELATIVE_RF
        matrix.postMult( inv );

    return( true );
}


// namespace osgwTools
}
