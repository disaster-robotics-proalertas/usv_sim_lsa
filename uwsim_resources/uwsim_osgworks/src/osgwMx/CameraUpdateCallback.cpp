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

#include <osgwMx/CameraUpdateCallback.h>
#include <osgwMx/MxCore.h>
#include <osgwMx/MxUtils.h>

#include <osg/Camera>


namespace osgwMx
{


CameraUpdateCallback::CameraUpdateCallback( MxCore* viewingCore )
  : _firstUpdate( true ),
    _mxCore( viewingCore )
{
}
CameraUpdateCallback::CameraUpdateCallback( const CameraUpdateCallback& rhs, const osg::CopyOp& copyop )
  : osg::NodeCallback( rhs, copyop ),
    _firstUpdate( rhs._firstUpdate ),
    _mxCore( rhs._mxCore )
{
}
CameraUpdateCallback::~CameraUpdateCallback()
{
}

void
CameraUpdateCallback::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
    if( _mxCore == NULL )
    {
        osg::notify( osg::WARN ) << "CameraUpdateCallback::operator(): _mxCore is NULL." << std::endl;
        return;
    }

    osg::Camera* cam = static_cast< osg::Camera* >( node );

    osg::BoundingSphere bs;
    if( _bs.valid() )
        bs = _bs;
    else
    {
        unsigned int idx;
        for( idx=0; idx<cam->getNumChildren(); idx++ )
        {
            if( idx==0 )
                bs = cam->getChild( idx )->getBound();
            else
                bs.expandBy( cam->getChild( idx )->getBound() );
        }
    }

    // Update the aspect ratio every frame.
    const osg::Viewport* vp = cam->getViewport();
    _mxCore->setAspect( vp->width() / vp->height() );

    if( _firstUpdate )
    {
        // Set the initial view.
        double distance = osgwMx::computeInitialDistanceFromFOVY( bs, _mxCore->getFovy() );
        _mxCore->setPosition( bs.center() - ( _mxCore->getDir() * distance ) );

        _mxCore->setInitialValues( _mxCore->getUp(),
            _mxCore->getDir(), _mxCore->getPosition() );

        _firstUpdate = false;
    }

    cam->setViewMatrix( _mxCore->getInverseMatrix() );
    cam->setProjectionMatrix( _mxCore->computeProjection(
        osgwMx::computeOptimalNearFar( _mxCore->getPosition(), bs, _mxCore->getOrtho() ) ) );

    traverse( node, nv );
}


// osgwMx
}
