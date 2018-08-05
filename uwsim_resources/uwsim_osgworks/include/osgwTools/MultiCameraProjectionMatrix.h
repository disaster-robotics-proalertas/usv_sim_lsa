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

#ifndef __OSGWORKS_OSGWTOOLS_MULTI_CAMERA_PROJECTION_MATRIX_H__
#define __OSGWORKS_OSGWTOOLS_MULTI_CAMERA_PROJECTION_MATRIX_H__ 1


#include <osgwTools/Export.h>
#include <osg/NodeCallback>
#include <OpenThreads/Mutex>

#include <set>


namespace osgwTools {



/** \class MultiCameraProjectionMatrix MultiCameraProjectionMatrix.h <osgwTools/MultiCameraProjectionMatrix.h>
\brief Compute a projection matrix with near & far encompassing two Cameras.
\details It is sometimes useful to use the same projection matrix between two
Cameras. OSG's auto-compute of the near & far planes can inhibit this, however,
as the computed near & far values consider the scene graph under only a single
Camera.

One example is a ray traced volume rendering algorithm drawn into a scene
containing other geometry. One Camera renders the normal geometry using an
FBO with a depth texture. The second Camera renders the volume, using the
previous Camer'a depth texture as input for correct z interaction. If the
projection matrices used by these two Cameras are different, visual artifacts
will result from incorrect z testing.

MultiCameraProjectionMatrix is a cull callback. To use it, attach it to the
subordinate post-render Camera. In the ray traced volume rendering example, the
root Camera renders the scene containing the regular geometry, and the volume
rendering Camera is a Node in the scene graph with render order set to
POST_RENDER. The app should add MultiCameraProjectionMatrix as a cull callback
to the volume rendering Camera.

MultiCameraProjectionMatrix maintains a set of NodeVisitor addresses. During cull,
if the NodeVisitor has never been encountered before, it is added to this set,
and the following steps occur:
 - If the subordinate Camera doesn't have a clamp projection matrix callback,
   one is created and added.
 - The CullVisitor's current Camera is assumed to be the root Camera. A clamp
   projection matrix callback is created and added to this Camera.

There could be multiple root Camera clamp callbacks, as in the case of tiled
rendering. In such a case, there are also multiple root Cameras. However, there
is only one subordinate Camera clamp callback.

The subordinate Camera clamp callback simply stores the near & far values in
a map indexed by the current thread ID.

The root Camera clamp callback obtains the near & far values of the subordinate
Camera (again using the current thread ID). It computes the maxima and minima
plane values for its scene and for the subordinate Camera's scene, which then
represent the near & far values that encompass the scenes of both Cameras.
Finally, it clamps the projection matrix using these near and far values.

For the scene rooted at the root Camera, the resulting projection matrix
is available for FFP rendering, as the gl_ProjectionMatrix uniform, and
also as a new uniform osgw_ProjectionMatrix.

For the scene rooted at the subordinate Camera, the FFP projection matrix
(and its associated gl_ProjectionMatrix uniform) are not valid. The app
must specify a shader program that uses osgw_ProjectionMatrix.

The uniform osgw_ProjectionMatrixInverse is also provided.

For best results, cull and draw should be executed from the same thread.
In osgViewer, that means SingleThreaded or CullDrawThreadPerContext
threading models. However, other threading models might work in some
specific cases. */
class OSGWTOOLS_EXPORT MultiCameraProjectionMatrix : public osg::NodeCallback
{
public:
    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv );

protected:
    typedef std::set< osg::ref_ptr<osg::NodeVisitor> > NVMap;
    NVMap _nvs;
    mutable OpenThreads::Mutex _mutex;
};


// namespace osgwTools
}

// __OSGWORKS_OSGWTOOLS_MULTI_CAMERA_PROJECTION_MATRIX_H__
#endif
