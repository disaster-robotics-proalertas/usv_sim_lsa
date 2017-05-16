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

#ifndef __OSGWMX_MX_EVENT_HANDLER_H__
#define __OSGWMX_MX_EVENT_HANDLER_H__ 1


#include <osgwMx/Export.h>
#include <osgwMx/MxCore.h>
#include <osgwMx/CameraUpdateCallback.h>

#include <osgGA/GUIEventHandler>
#include <osg/Node>


namespace osgwMx
{


/** \class MxEventHandler MxEventHandler.h <osgwMx/MxEventHandler.h>
\brief An osgGA interface to the MxCore.
This class receives events from the osgGA event queue and passes them
to an instance of the MxCore, which is responsible for maintaining
view and projection parameters.

This class also has a CameraUpdateCallback. At init time, your application
must obtain this callback and attach it as an update callback to your Camera
node. The callback obtains the view and projection matrices from the
MxCore and sets them in the Camera node during the update traversal.

If you wanted to avoid using osgGA, you'd rewrite this class in Qt or Wx,
for example. You could still use the "has a" design pattern to keep instances
of the MxCore and CameraUpdateCallback objects, but you'd receive your
own events and make calls into MxCore using this class's handle() function
as a template.

\section User Interface

\li Shift + leftmouse drag: pan.
\li Ctrl + leftmouse drag: turn head.
\li Leftmouse drag: trackball-style orbit.
\li Ctrl + shift + leftmouse click: Pick trackball center point.
\li Shift + rightmouse drag: Move left-right and up-down.
\li Rightmouse drag: Move forward-backward (dolly).
\li Scroll wheel: field of view.
\li 'o': Toggle between orthographic and perspective (default).
\li 'd': Display current view yaw/pitch/roll values.
\li ' ': Return to home position.

*/
class OSGWMX_EXPORT MxEventHandler : public osgGA::GUIEventHandler
{
public:
    MxEventHandler();
    MxEventHandler( const MxEventHandler& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgwMx,MxEventHandler);

    // Base class support.
    virtual bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );


    void setSceneData( osg::Node* scene ) { _scene = scene; }
    void setWorldUpVector( const osg::Vec3d& worldUp ) { _worldUp = worldUp; }

    /** Set the MxCore. This allows sharing an MxCore between multiple device
    handlers, such as MxEventHandler and MxGamePad. */
    void setMxCore( osgwMx::MxCore* mxCore );
    /** Get the MxCore for access to the managed matrix. */
    MxCore* getMxCore();

    osg::NodeCallback* getGenericMatrixCallback();
    CameraUpdateCallback* getMatrixCallback();

protected:
    ~MxEventHandler();

    osg::ref_ptr< MxCore > _mxCore;
    osg::ref_ptr< osg::Node > _scene;
    osg::ref_ptr< CameraUpdateCallback > _cameraUpdateCallback;

    osg::Vec3d _worldUp;

    double _lastX, _lastY;
    float _lastXPixel, _lastYPixel;
    bool _leftDragging, _leftClick;

    osg::Vec4d _panPlane;
    double _moveScale;
};


// osgwMx
}


// __OSGWMX_MX_EVENT_HANDLER_H__
#endif
