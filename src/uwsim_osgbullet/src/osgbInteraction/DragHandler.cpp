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

#include <osgbInteraction/DragHandler.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbCollision/Utils.h>
#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/PhysicsThread.h>

#include <osgGA/GUIEventHandler>
#include <osg/Camera>
#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>

#include <btBulletDynamicsCommon.h>

#include <osg/io_utils>
#include <sstream>


namespace osgbInteraction
{


DragHandler::DragHandler( btDynamicsWorld* dw, osg::Camera* scene )
  : _dw( dw ),
    _scene( scene ),
    _constraint( NULL ),
    _constrainedMotionState( NULL ),
    _pt( NULL )
{
}
DragHandler::~DragHandler()
{
}

bool DragHandler::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    const bool ctrl( ( ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL ) != 0 );

    if( ea.getEventType() == osgGA::GUIEventAdapter::PUSH )
    {
        if( !ctrl ||
            ( ( ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON ) == 0 ) )
            return( false );

        const bool picked = pick( ea.getXnormalized(), ea.getYnormalized() );

        if( picked )
            _constraint->getRigidBodyA().activate( true );

        return( picked );
    }
    else if( ea.getEventType() == osgGA::GUIEventAdapter::DRAG )
    {
        if( ( !ctrl ) || ( _constraint == NULL ) ||
            ( ( ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON ) == 0 ) )
            return( false );

        osg::Vec4d farPointNDC = osg::Vec4d( ea.getXnormalized(), ea.getYnormalized(), 1., 1. );
        osg::Matrix p = _scene->getProjectionMatrix();
        double zNear, zFar, fovy, aspect;
        p.getPerspective( fovy, aspect, zNear, zFar );
        osg::Vec4d farPointCC = farPointNDC * zFar;
        p.invert( p );
        osg::Matrix v = _scene->getViewMatrix();
        v.invert( v );
        osg::Vec4d farPointWC = farPointCC * p * v;

        osg::Vec3d look, at, up;
        _scene->getViewMatrixAsLookAt( look, at, up );


        // Intersect ray with plane.
        // TBD. Stolen from osgWorks' MxCore::intersectPlaneRay(), which really should be in some math library somewhere.
        osg::Vec3d planeNormal = osg::Vec3d( _dragPlane[ 0 ], _dragPlane[ 1 ], _dragPlane[ 2 ] );
        const osg::Vec3d vDir = osg::Vec3( farPointWC[ 0 ], farPointWC[ 1 ], farPointWC[ 2 ] ) - look;
        const double dotVd = vDir * planeNormal;
        if( dotVd == 0. )
        {
            osg::notify( osg::WARN ) << "DragHandler: No plane intersection." << std::endl;
            return( false );
        }
        double length = -( planeNormal * look + _dragPlane[ 3 ] ) / dotVd;
        osg::Vec3 pointOnPlane = look + ( vDir * length );
        osg::notify( osg::DEBUG_FP ) << "  OSG point " << pointOnPlane << std::endl;

        if( _pt != NULL )
            _pt->pause( true );

        osg::Matrix ow2bw;
        if( _constrainedMotionState != NULL )
            ow2bw = _constrainedMotionState->computeOsgWorldToBulletWorld();
        osg::Vec3d bulletPoint = pointOnPlane * ow2bw;
        osg::notify( osg::DEBUG_FP ) << "    bullet point " << bulletPoint << std::endl;

        _constraint->setPivotB( osgbCollision::asBtVector3( bulletPoint ) );

        if( _pt != NULL )
            _pt->pause( false );

        return( true );
    }
    else if( ea.getEventType() == osgGA::GUIEventAdapter::RELEASE )
    {
        if( _constraint == NULL )
            return( false );

        if( _pt != NULL )
            _pt->pause( true );

        _dw->removeConstraint( _constraint );

        if( _pt != NULL )
            _pt->pause( false );

        delete _constraint;
        _constraint = NULL;
        _constrainedMotionState = NULL;
        return( true );
    }

    return( false );
}

void DragHandler::setThreadedPhysicsSupport( osgbDynamics::PhysicsThread* pt )
{
    _pt = pt;
}

bool DragHandler::pick( float wx, float wy )
{
    if( !( _scene.valid() ) )
    {
        osg::notify( osg::WARN ) << "DragHandler: _scene == NULL." << std::endl;
        return( false );
    }

    osg::Viewport* vp = _scene->getViewport();
    float mx = vp->x() + (int)( (float)vp->width() * ( wx * 0.5f + 0.5f ) );
    float my = vp->y() + (int)( (float)vp->height() * ( wy * 0.5f + 0.5f ) );

    osgUtil::LineSegmentIntersector* intersector = new osgUtil::LineSegmentIntersector(
        osgUtil::Intersector::WINDOW, mx, my );
    osgUtil::IntersectionVisitor intersectVisitor( intersector, NULL );
    _scene->accept( intersectVisitor );

    osgUtil::LineSegmentIntersector::Intersections& intersections = intersector->getIntersections();

    osg::Vec3d pickPointWC;
    osgbCollision::RefRigidBody* rrb( NULL );
    osgUtil::LineSegmentIntersector::Intersections::const_iterator it;
    for( it = intersections.begin(); it != intersections.end(); ++it )
    {
        const osgUtil::LineSegmentIntersector::Intersection& intersection = *it;
        const osg::NodePath& np = intersection.nodePath;
        osg::NodePath::const_reverse_iterator rnpit;
        for( rnpit = np.rbegin(); rnpit != np.rend(); ++rnpit )
        {
            const osg::Node& currentNode = *( *rnpit );

            //if we have a callback
            //    call the callback
            //else
            {
                // Default behavior. See if UserData is a RefRigidBody.
                const osg::Referenced* userData = currentNode.getUserData();
                rrb = const_cast< osgbCollision::RefRigidBody* >(
                    dynamic_cast< const osgbCollision::RefRigidBody* >( userData ) );
                if( rrb != NULL )
                    break;
            }
        }
        if( rrb != NULL )
        {
            // TBD. Hm. Maybe need this in local coords?
            pickPointWC = intersection.getWorldIntersectPoint();
            break;
        }
    }
    if( rrb == NULL )
        return( false );

    btRigidBody* rb = rrb->get();

    // Save the MotionState for this rigid body. We'll use it during the DRAG events.
    _constrainedMotionState = dynamic_cast< osgbDynamics::MotionState* >( rb->getMotionState() );
    osg::Matrix ow2col;
    if( _constrainedMotionState != NULL )
        ow2col = _constrainedMotionState->computeOsgWorldToCOLocal();
    osg::Vec3d pickPointBulletOCLocal = pickPointWC * ow2col;
    osg::notify( osg::DEBUG_FP ) << "pickPointWC: " << pickPointWC << std::endl;
    osg::notify( osg::DEBUG_FP ) << "pickPointBulletOCLocal: " << pickPointBulletOCLocal << std::endl;

    // We now have the intersetionPoint and a pointer to the btRigidBody.
    // Make a Bullet point-to-point constraint, so we can drag it around.
    _constraint = new btPoint2PointConstraint( *rb,
        osgbCollision::asBtVector3( pickPointBulletOCLocal ) );
    if( _pt != NULL )
        _pt->pause( true );
    _dw->addConstraint( _constraint );
    if( _pt != NULL )
        _pt->pause( false );


    // Also make a drag plane.
    osg::Vec3d look, at, up;
    _scene->getViewMatrixAsLookAt( look, at, up );
    osg::Vec3d viewDir = at - look;
    viewDir.normalize();
    _dragPlane = osg::Vec4( viewDir, -( pickPointWC * viewDir ) );

    return( true );
}


// osgbInteraction
}
