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

#include <osgbInteraction/LaunchHandler.h>
#include <btBulletDynamicsCommon.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgwTools/AbsoluteModelTransform.h>
#include <osgbCollision/Utils.h>
#include <osgbDynamics/PhysicsThread.h>
#include <osgbDynamics/TripleBuffer.h>
#include <osg/Group>
#include <osg/Camera>
#include <osg/Geode>
#include <osgwTools/Shapes.h>


namespace osgbInteraction
{


LaunchHandler::LaunchHandler( btDynamicsWorld* dw, osg::Group* attachPoint, osg::Camera* camera )
  : _dw( dw ),
    _attachPoint( attachPoint ),
    _camera( camera ),
    _launchCollisionShape( NULL ),
    _initialVelocity( 10. ),
    _group( 0 ),
    _mask( 0 ),
    _pt( NULL ),
    _tb( NULL ),
    _msl( NULL )
{
    // Make the default launch model: Sphere with radius 1.0.
    osg::Geode* geode = new osg::Geode;
    const double radius( 1.0 );
    geode->addDrawable( osgwTools::makeGeodesicSphere( radius ) );
    _launchModel = geode;
    _launchCollisionShape = new btSphereShape( radius );
    _ownsCollisionShape = true;
}

LaunchHandler::~LaunchHandler()
{
    reset();
    if( ( _launchCollisionShape != NULL ) && _ownsCollisionShape )
        delete _launchCollisionShape;
}

void LaunchHandler::setThreadedPhysicsSupport( osgbDynamics::PhysicsThread* pt, osgbDynamics::TripleBuffer* tb, osgbDynamics::MotionStateList* msl )
{
    _pt = pt;
    _tb = tb;
    _msl = msl;
}

void LaunchHandler::setLaunchModel( osg::Node* model, btCollisionShape* shape )
{
    _launchModel = model;

    if( ( _launchCollisionShape != NULL ) && _ownsCollisionShape )
        delete _launchCollisionShape;

    if( shape != NULL )
    {
        _launchCollisionShape = shape;
        _ownsCollisionShape = false;
    }
    else
    {
        btConvexHullShape* ch = osgbCollision::btConvexHullCollisionShapeFromOSG( model );
        ch->setMargin( 0.0 );
        _launchCollisionShape = ch;
        _ownsCollisionShape = true;
    }
}

bool LaunchHandler::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& )
{
    // We want a shift-leftmouse event. Return false if we don't have it.
    if( ( ea.getEventType() != osgGA::GUIEventAdapter::PUSH ) ||
        ( ea.getButton() != osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON ) ||
        ( ( ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SHIFT ) == 0 ) )
        return( false );

    osg::Matrix view = _camera->getViewMatrix();
    osg::Vec3 look, at, up;
    view.getLookAt( look, at, up );

    osg::Matrix proj = _camera->getProjectionMatrix();
    double fovy, aspect, zNear, zFar;
    proj.getPerspective( fovy, aspect, zNear, zFar );

    view.invert( view );
    proj.invert( proj );
    osg::Vec4 clip( ea.getXnormalized() * zFar, ea.getYnormalized() * zFar, zFar, zFar );
    osg::Vec4 wc = clip * proj * view;

    const osg::Vec3 launchPos = look + ( up * ( _launchModel->getBound()._radius * 2. ) );

    osg::Matrix parentTrans = osg::Matrix::translate( launchPos );
    osg::Vec3 launchDir = osg::Vec3( wc[0], wc[1], wc[2] ) - launchPos;
    launchDir.normalize();

    osg::ref_ptr< osgwTools::AbsoluteModelTransform > amt = new osgwTools::AbsoluteModelTransform;
    amt->setDataVariance( osg::Object::DYNAMIC );
    amt->addChild( _launchModel.get() );

    _attachPoint->addChild( amt.get() );
    _nodeList.push_back( amt.get() );

    osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = amt.get();
    cr->_mass = 1.;
    cr->_parentTransform = parentTrans;
    btRigidBody* rb = osgbDynamics::createRigidBody( cr.get(), _launchCollisionShape );
    rb->setLinearVelocity( osgbCollision::asBtVector3( launchDir * _initialVelocity ) );
    rb->setAngularVelocity( btVector3( .2, .3, 1.5 ) );

    osgbDynamics::MotionState* motion = static_cast< osgbDynamics::MotionState* >( rb->getMotionState() );
    if( _tb != NULL )
        motion->registerTripleBuffer( _tb );
    if( _msl != NULL )
        _msl->insert( motion );

    if( _pt != NULL )
        _pt->pause( true );

    bool added( false );
    amt->setUserData( new osgbCollision::RefRigidBody( rb ) );
    if( (_group != 0) || (_mask != 0) )
    {
        // Collision filters were specified. Get the discrete dynamics world.
        btDiscreteDynamicsWorld* ddw = dynamic_cast< btDiscreteDynamicsWorld* >( _dw );
        if( ddw != NULL )
        {
            ddw->addRigidBody( rb, _group, _mask );
            added = true;
        }
    }
    if( !added )
    {
        // This is both the main path for not using collision filters, and
        // also the fallback if the btDiscreteDynamicsWorld* dynamic cast fails.
        _dw->addRigidBody( rb );
    }

    if( _pt != NULL )
        _pt->pause( false );

    return( true );
}

void LaunchHandler::reset()
{
    if( _pt != NULL )
        _pt->pause( true );

    NodeList::iterator it;
    for( it=_nodeList.begin(); it != _nodeList.end(); ++it )
    {
        osg::ref_ptr< osg::Node > node = *it;
        osgbCollision::RefRigidBody* rrb = dynamic_cast< osgbCollision::RefRigidBody* >( node->getUserData() );
        if( rrb == NULL )
        {
            osg::notify( osg::WARN ) << "LaunchHandler::reset: Node has no RefRigidBody in UserData." << std::endl;
            continue;
        }

        btRigidBody* rb = rrb->get();
        if( rb->getMotionState() )
        {
            osgbDynamics::MotionState* motion = static_cast< osgbDynamics::MotionState* >( rb->getMotionState() );
            if( _msl != NULL )
            {
                osgbDynamics::MotionStateList::iterator it = _msl->find( motion );
                if( it != _msl->end() )
                    _msl->erase( it );
            }
            delete motion;
        }
        _dw->removeRigidBody( rb );
        delete rb;
        _attachPoint->removeChild( node.get() );
    }

    if( _pt != NULL )
        _pt->pause( false );

    _nodeList.clear();
}


// osgbInteraction
}
