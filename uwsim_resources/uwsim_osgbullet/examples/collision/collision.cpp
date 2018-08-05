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

#include <btBulletCollisionCommon.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/Utils.h>

#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osgGA/GUIEventHandler>
#include <osg/MatrixTransform>
#include <osg/Geode>
#include <osgwTools/Shapes.h>
#include <osgwTools/Version.h>

#include <osg/io_utils>
#include <iostream>


/* \cond */
class MoveManipulator : public osgGA::GUIEventHandler
{
public:
    MoveManipulator() : _co( NULL ), _mt( NULL ) {}
    MoveManipulator( const MoveManipulator& mm, osg::CopyOp copyop ) : _co( mm._co ), _mt( mm._mt ) {}
    ~MoveManipulator() {}
#if( OSGWORKS_OSG_VERSION > 20800 )
    META_Object(osgBulletExample,MoveManipulator);
#endif

    virtual bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if( ( ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL ) == 0 )
        {
            return( false );
        }
        else if( ea.getEventType() == osgGA::GUIEventAdapter::PUSH )
        {
            _lastX = ea.getXnormalized();
            _lastY = ea.getYnormalized();
            return( true );
        }
        else if( ea.getEventType() == osgGA::GUIEventAdapter::DRAG )
        {
            double deltaX = ea.getXnormalized() - _lastX;
            double deltaY = ea.getYnormalized() - _lastY;
            _lastX = ea.getXnormalized();
            _lastY = ea.getYnormalized();

            deltaX *= 6.;
            deltaY *= 6.;
            osg::Matrix trans = osgbCollision::asOsgMatrix( _co->getWorldTransform() );
            trans = trans * osg::Matrix::translate( deltaX, 0., deltaY );
            _mt->setMatrix( trans );
            _co->setWorldTransform( osgbCollision::asBtTransform( trans ) );
            return( true );
        }
        return( false );
    }

    void setCollisionObject( btCollisionObject* co ) { _co = co; }
    void setMatrixTransform( osg::MatrixTransform* mt ) { _mt = mt; }

protected:
    btCollisionObject* _co;
    osg::MatrixTransform* _mt;
    double _lastX, _lastY;
};
/* \endcond */


btCollisionWorld* initCollision()
{
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher( collisionConfiguration );

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface* inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btCollisionWorld* collisionWorld = new btCollisionWorld( dispatcher, inter, collisionConfiguration );

    return( collisionWorld );
}


osg::Node* createScene( btCollisionWorld* cw, MoveManipulator* mm, osg::ArgumentParser& arguments )
{
    osg::ref_ptr< osg::Group > root = new osg::Group;

    // Create a static box
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeBox( osg::Vec3( .5, .5, .5 ) ) );
    root->addChild( geode );

    btCollisionObject* btBoxObject = new btCollisionObject;
    btBoxObject->setCollisionShape( osgbCollision::btBoxCollisionShapeFromOSG( geode ) );
    btBoxObject->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
    cw->addCollisionObject( btBoxObject );


    // Create a box we can drag around with the mouse
    geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeBox( osg::Vec3( .5, .5, .5 ) ) );
    osg::Matrix transMatrix = osg::Matrix::translate( 4., 0., 0. );
    osg::MatrixTransform* mt = new osg::MatrixTransform( transMatrix );
    mt->addChild( geode );
    root->addChild( mt );

    btBoxObject = new btCollisionObject;
    btBoxObject->setCollisionShape( osgbCollision::btBoxCollisionShapeFromOSG( geode ) );
    btBoxObject->setCollisionFlags( btCollisionObject::CF_KINEMATIC_OBJECT );
    btBoxObject->setWorldTransform( osgbCollision::asBtTransform( transMatrix ) );
    cw->addCollisionObject( btBoxObject );
    mm->setCollisionObject( btBoxObject );
    mm->setMatrixTransform( mt );


    return( root.release() );
}

void detectCollision( bool& lastColState, btCollisionWorld* cw )
{
    unsigned int numManifolds = cw->getDispatcher()->getNumManifolds();
    if( ( numManifolds == 0 ) && (lastColState == true ) )
    {
        osg::notify( osg::ALWAYS ) << "No collision." << std::endl;
        lastColState = false;
    }
    else {
        for( unsigned int i = 0; i < numManifolds; i++ )
        {
            btPersistentManifold* contactManifold = cw->getDispatcher()->getManifoldByIndexInternal(i);
            unsigned int numContacts = contactManifold->getNumContacts();
            for( unsigned int j=0; j<numContacts; j++ )
            {
                btManifoldPoint& pt = contactManifold->getContactPoint( j );
                if( ( pt.getDistance() <= 0.f ) && ( lastColState == false ) )
                {
                    // grab these values for the contact normal arrows:
                    osg::Vec3 pos = osgbCollision::asOsgVec3( pt.getPositionWorldOnA() ); // position of the collision on object A
                    osg::Vec3 normal = osgbCollision::asOsgVec3( pt.m_normalWorldOnB ); // returns a unit vector
                    float pen = pt.getDistance(); //penetration depth

                    osg::Quat q;
                    q.makeRotate( osg::Vec3( 0, 0, 1 ), normal );

                    osg::notify( osg::ALWAYS ) << "Collision detected." << std::endl;

                    osg::notify( osg::ALWAYS ) << "\tPosition: " << pos << std::endl;
                    osg::notify( osg::ALWAYS ) << "\tNormal: " << normal << std::endl;
                    osg::notify( osg::ALWAYS ) << "\tPenetration depth: " << pen << std::endl;
                    //osg::notify( osg::ALWAYS ) << q.w() <<","<< q.x() <<","<< q.y() <<","<< q.z() << std::endl;
                    lastColState = true;
                }
                else if( ( pt.getDistance() > 0.f ) && ( lastColState == true ) )
                {
                    osg::notify( osg::ALWAYS ) << "No collision." << std::endl;
                    lastColState = false;
                }
            }
        }
    }
}

int main( int argc,
         char * argv[] )
{
    btCollisionWorld* collisionWorld = initCollision();

    osg::ArgumentParser arguments( &argc, argv );
    MoveManipulator* mm = new MoveManipulator;
    osg::ref_ptr< osg::Node > root = createScene( collisionWorld, mm, arguments );

    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 10, 30, 800, 600 );
    viewer.setCameraManipulator( new osgGA::TrackballManipulator() );
    viewer.addEventHandler( mm );
    viewer.setSceneData( root.get() );

    bool lastColState = false;
    while( !viewer.done() )
    {
        collisionWorld->performDiscreteCollisionDetection();

        detectCollision( lastColState, collisionWorld );

        viewer.frame();
    }

    return( 0 );
}



/** \page collision Using osgBullet For Collision Detection
osgBullet consists of two libraries, osgbCollision and osgbDynamics. This
library split allows your application to use Bullet for collision detection
with no dependency on libBulletDynamics, and render your results with OSG.
osgBullet contains an example program, \c collision, to demonstrate this usage.

\c collision renders two boxes. You can view them from any angle using the
OSG TrackballManipulator, but the example behaves more intuitively if you
use the default home position.

Move the box on the right by holding down the control key and dragging
with your left mouse button. If you drag the right box so that it is in
collision with the left box, the following message appears on the console:

\code
Collision detected.
        Position: 0.5 0.5 0.5
        Normal: -1 -0 -0
        Penetration depth: -5.96046e-008
\endcode

The \c Position, \c normal, and \c Penetration \c depth values are taken from the
Bullet collision information.

Drag the right box away from the left box and the following message appears
on the console:

\code
No collision.
\endcode

Using osgBullet, your application interfaces directly with
the Bullet API to determine if a collision has occurred, and if so, which
collision objects are in collision. The \c collision
example detects collisions by examining the manifold count in the Bullet collision
dispatcher, but Bullet provides other ways to detect collisions, as
discussed in the Bullet documentation and online forum.
*/
