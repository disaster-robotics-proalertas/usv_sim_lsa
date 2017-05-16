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

#ifndef __OSGWMX_CAMERA_UPDATE_CALLBACK_H__
#define __OSGWMX_CAMERA_UPDATE_CALLBACK_H__ 1


#include <osgwMx/Export.h>
#include <osgwMx/MxCore.h>

#include <osg/NodeCallback>


namespace osgwMx
{


/** \class CameraUpdateCallback CameraUpdateCallback.h <osgwMx/CameraUpdateCallback.h>
\brief A Camera update callback that modifies both the view and projection matrices.
*/
class OSGWMX_EXPORT CameraUpdateCallback : public osg::NodeCallback
{
public:
    CameraUpdateCallback( MxCore* viewingCore=NULL );
    CameraUpdateCallback( const CameraUpdateCallback& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgwMx,CameraUpdateCallback);

    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv );

    /** By default, CameraUpdateCallback automatically computes an initial view distance
    from the scene graph bound center than encompasses the entire view. Call this function
    with 'false' before the first frame to disable auto computation of the view. Or, call
    it after rendering has begun, passing 'true' as a parameter, to jump to the initial
    view distance. */
    void enableAutoComputeInitialView( bool enable ) { _firstUpdate = enable; }
    /** When CameraUpdateCallback computes the initial view distance, it bases that
    computation on the bounding volumes of the Camera's children. Call this function
    before the first frame to pass in an arbitrary bounding volume for initial view
    computation. */
    void useBound( const osg::BoundingSphere& bs ) { _bs = bs; }

protected:
    ~CameraUpdateCallback();

    bool _firstUpdate;
    osg::BoundingSphere _bs;

    osg::ref_ptr< MxCore > _mxCore;
};


// osgwMx
}


// __OSGWMX_CAMERA_UPDATE_CALLBACK_H__
#endif
