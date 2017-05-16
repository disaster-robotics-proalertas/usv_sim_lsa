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

#ifndef __OSGBINTERACTION_DRAG_HANDLER_H__
#define __OSGBINTERACTION_DRAG_HANDLER_H__ 1


#include <osgbInteraction/Export.h>
#include <osgGA/GUIEventHandler>
#include <osgbDynamics/MotionState.h>
#include <btBulletDynamicsCommon.h>
#include <osg/Vec4>


// Forward
namespace osg {
    class Camera;
}

namespace osgbDynamics {
    class PhysicsThread;
}


namespace osgbInteraction
{


/** \class DragHandler DragHandler.h <osgbInteraction\DragHandler.h>
\brief An event handler for selecting and dragging rigid bodies.

To use this event handler, simply create an instance of it and add it to your
osgViewer::Viewer.

During a ctrl-leftmouse click, DragHandler does an intersection test with
the \c scene. The test succeeds if the picked object has a Node in its NodePath
containing a RefRigidBody stored in the Node's UserData. DragHandler then adds
the picked btRigidBody to a new btPoint2PointConstraint and adds it to the
dynamics world. DragHandler also computes a drag plane, orthogonal to the view
direction and containing the intersection point.

On subsequent ctrl-leftmouse drag events, DragHandler back-transform the mouse
position to create a world space ray, and intersects it with the DragPlane.
DragHandler then sets this intersection point in the constraint.

On a leftmouse release event, DragHandler removes the constraint and deletes it.
*/
class OSGBINTERACTION_EXPORT DragHandler : public osgGA::GUIEventHandler
{
public:
    /** \brief Constructor.
    \param dw The Bullet dynamics world. When the DragHandler creates a
    btPoint2PointConstraint, it adds it to this dynamics world.
    \param scene Scene graph used for picking. \c scene must be a Camera node
    to allow DragHandler to properly convert from window to world coordinates
    during selection and dragging. */
    DragHandler( btDynamicsWorld* dw, osg::Camera* scene );

    /** \brief Handle events.

    Controls:
    \li ctrl-left-mouse Select and drag a rigid body.
    */
    virtual bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa );

    /** \brief Support for running the Bullet physics simultation in a separate thread.

    Call this function to specify the osgbDynamics::PhysicsThread. DragHandler pauses
    and unpauses the thread during constraint creation, modification, and deletion. */
    void setThreadedPhysicsSupport( osgbDynamics::PhysicsThread* pt );

protected:
    ~DragHandler();

    /** \brief Picking support.
    \param wx Normalized (-1.0 to 1.0) x mouse position
    \param wy Normalized (-1.0 to 1.0) Y mouse position
    */
    bool pick( float wx, float wy );

    btDynamicsWorld* _dw;
    osg::ref_ptr< osg::Camera > _scene;

    btPoint2PointConstraint* _constraint;
    osgbDynamics::MotionState* _constrainedMotionState;
    osg::Vec4 _dragPlane;

    osgbDynamics::PhysicsThread* _pt;
};


// osgbInteraction
}


// __OSGBINTERACTION_DRAG_HANDLER_H__
#endif