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

#include <osgbDynamics/MotionState.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbDynamics/RigidBodyAnimation.h>
#include <btBulletDynamicsCommon.h>

#include <osgwTools/Version.h>

#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgGA/TrackballManipulator>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/AnimationPath>
#include <osg/PolygonMode>
#include <osg/PolygonOffset>
#include <osg/io_utils>
#include <iostream>


osg::AnimationPath * createAnimationPath( const osg::Vec3 & origin,
                                          const osg::Vec3 & direction,
                                          double duration )
{
    osg::AnimationPath * animationPath = new osg::AnimationPath;

    animationPath->setLoopMode( osg::AnimationPath::LOOP );

    int numSamples( 40 );
    //double time( 0. );
    osg::Quat orient;
    int idx;
    for( idx = 0; idx < numSamples; idx++ )
    {
        const double scale( ( double )idx / ( double )numSamples );
        const double curTime( duration * scale );
        osg::Vec3 pos( origin + ( direction * scale ) );

        animationPath->insert( curTime, osg::AnimationPath::ControlPoint( pos, orient ) );
    }

    return( animationPath );
}

osg::MatrixTransform * createAnimatedObject( osg::Vec3 size )
{
    osg::Box * box = new osg::Box();

    box->setHalfLengths( size );

    osg::ShapeDrawable * shape = new osg::ShapeDrawable( box );

    osg::Geode * geode = new osg::Geode();
    geode->addDrawable( shape );

    osg::MatrixTransform* t0 = new osg::MatrixTransform();
    t0->setMatrix( osg::Matrix::translate( .25, -.25, -.25 ) );
    t0->addChild( geode );
    osg::MatrixTransform* t1 = new osg::MatrixTransform();
    t1->setMatrix( osg::Matrix::translate( -.25, .25, .25 ) );
    t1->addChild( geode );
    
    osg::MatrixTransform* transform = new osg::MatrixTransform();
    transform->addChild( t0 );
    transform->addChild( t1 );

    return( transform );
}

osg::MatrixTransform * createOSGBox( osg::Vec3 size )
{
    osg::Box * box = new osg::Box();

    box->setHalfLengths( size );

    osg::ShapeDrawable * shape = new osg::ShapeDrawable( box );

    osg::Geode * geode = new osg::Geode();
    geode->addDrawable( shape );

    osg::MatrixTransform * transform = new osg::MatrixTransform();
    transform->addChild( geode );

    return( transform );
}

osg::MatrixTransform * createOffOriginOSGBox( osg::Vec3 size )
{
    const osg::Vec3 dim( size * 2 );

    osg::Geode * geode = new osg::Geode;
    osg::Geometry * geom = new osg::Geometry;
    osg::Vec3Array * v = new osg::Vec3Array;

    v->resize( 8 );
    ( *v )[ 0 ] = osg::Vec3( 0, 0, 0 );
    ( *v )[ 1 ] = osg::Vec3( 0, dim.y(), 0 );
    ( *v )[ 2 ] = osg::Vec3( dim.x(), dim.y(), 0 );
    ( *v )[ 3 ] = osg::Vec3( dim.x(), 0, 0 );
    ( *v )[ 4 ] = osg::Vec3( 0, 0, dim.z() );
    ( *v )[ 5 ] = osg::Vec3( dim.x(), 0, dim.z() );
    ( *v )[ 6 ] = osg::Vec3( dim.x(), dim.y(), dim.z() );
    ( *v )[ 7 ] = osg::Vec3( 0, dim.y(), dim.z() );
    geom->setVertexArray( v );

    osg::Vec3Array * n = new osg::Vec3Array;
    n->resize( 8 );
    ( *n )[ 0 ] = osg::Vec3( 0, 0, -1 );
    ( *n )[ 1 ] = osg::Vec3( 0, 0, 1 );
    ( *n )[ 2 ] = osg::Vec3( -1, 0, 0 );
    ( *n )[ 3 ] = osg::Vec3( 1, 0, 0 );
    ( *n )[ 4 ] = osg::Vec3( 0, -1, 0 );
    ( *n )[ 5 ] = osg::Vec3( 0, 1, 0 );
    ( *n )[ 6 ] = osg::Vec3( 0, 1, 0 );
    ( *n )[ 7 ] = osg::Vec3( 0, 1, 0 );
    geom->setNormalArray( n );
#if( OSGWORKS_OSG_VERSION < 30109 )
    geom->setNormalBinding( osg::Geometry::BIND_PER_PRIMITIVE );
#else
    // TBD need to fix this for current OSG to render a box properly.
    geom->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
#endif

    osg::Vec4Array * c = new osg::Vec4Array;
    c->resize( 8 );
    ( *c )[ 0 ] = osg::Vec4( 1, 1, 1, 1 );
    ( *c )[ 1 ] = osg::Vec4( .6, 0, 0, 1 );
    ( *c )[ 2 ] = osg::Vec4( .6, 0, 0, 1 );
    ( *c )[ 3 ] = osg::Vec4( .6, 0, 0, 1 );
    ( *c )[ 4 ] = osg::Vec4( .6, 0, 0, 1 );
    ( *c )[ 5 ] = osg::Vec4( .6, 0, 0, 1 );
    ( *c )[ 6 ] = osg::Vec4( .6, 0, 0, 1 );
    ( *c )[ 7 ] = osg::Vec4( .6, 0, 0, 1 );
    geom->setColorArray( c );
    geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

    GLushort indices[] =
    {
        0, 1, 2, 3,
        4, 5, 6, 7,
        0, 4, 7, 1,
        3, 2, 6, 5,
        0, 3, 5, 4,
        1, 7, 6, 2
    };
    geom->addPrimitiveSet( new osg::DrawElementsUShort( GL_QUADS, 24, indices ) );
    geode->addDrawable( geom );

    osg::MatrixTransform * mt = new osg::MatrixTransform();
    mt->addChild( geode );
    return( mt );
}

btRigidBody * createBTBox( osg::MatrixTransform * box,
                          osg::Vec3 center )
{
    btCollisionShape * collision = osgbCollision::btBoxCollisionShapeFromOSG( box );

    osgbDynamics::MotionState * motion = new osgbDynamics::MotionState();
    motion->setTransform( box );
    motion->setParentTransform( osg::Matrix::translate( center ) );

    btScalar mass( 0. );
    btVector3 inertia( 0, 0, 0 );
    btRigidBody::btRigidBodyConstructionInfo rb( mass, motion, collision, inertia );
    btRigidBody * body = new btRigidBody( rb );

    return( body );
}

btRigidBody* createBTCompound( osg::MatrixTransform* obj,
                              osg::Vec3 center )
{
    btCollisionShape* collision = osgbCollision::btTriMeshCollisionShapeFromOSG( obj );

    osgbDynamics::MotionState * motion = new osgbDynamics::MotionState();
    motion->setTransform( obj );
    motion->setParentTransform( osg::Matrix::translate( center ) );

    btScalar mass( 0. );
    btVector3 inertia( 0, 0, 0 );
    btRigidBody::btRigidBodyConstructionInfo rb( mass, motion, collision, inertia );
    btRigidBody * body = new btRigidBody( rb );

    osg::Node* n = osgbCollision::osgNodeFromBtCollisionShape( collision );
    obj->addChild( n );
        osg::StateSet* state = n->getOrCreateStateSet();
        osg::PolygonMode* pm = new osg::PolygonMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
        state->setAttributeAndModes( pm );
        osg::PolygonOffset* po = new osg::PolygonOffset( -1, -1 );
        state->setAttributeAndModes( po );
        state->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    return( body );
}

btDynamicsWorld * initPhysics()
{
    btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    dynamicsWorld->setGravity( btVector3( 0, 0, -10 ) );

    return( dynamicsWorld );
}

void createTarget( osg::Group * root,
                   btDynamicsWorld * dynamicsWorld )
{
    osg::MatrixTransform * target = createOffOriginOSGBox( osg::Vec3( 1, 1, 1 ) );
    root->addChild( target );

    /* osgBullet code */
    osgbDynamics::MotionState * motion = new osgbDynamics::MotionState();
    motion->setTransform( target );
    motion->setCenterOfMass( osg::Vec3( 1, 1, 1 ) );

    btCollisionShape * collision = osgbCollision::btBoxCollisionShapeFromOSG( target );


    btScalar mass( 1.0 );
    btVector3 inertia;
    collision->calculateLocalInertia( mass, inertia );
    btRigidBody::btRigidBodyConstructionInfo rbinfo( mass, motion, collision, inertia );
    btRigidBody * body = new btRigidBody( rbinfo );
    body->setLinearVelocity( btVector3( 1, 0, 0 ) );
    body->setAngularVelocity( btVector3( 1, 0, 0 ) );
    dynamicsWorld->addRigidBody( body );
}

int main( int argc,
          char * argv[] )
{
    osg::ref_ptr< osg::Group > root = new osg::Group();

    btDynamicsWorld * dynamicsWorld = initPhysics();


    // Create target
    createTarget( root.get(), dynamicsWorld );


    // Make the ground plane
    osg::MatrixTransform * ground = createOSGBox( osg::Vec3( 10, 10, .01 ) );
    root->addChild( ground );
    btRigidBody* groundBody = createBTBox( ground, osg::Vec3( 0, 0, -10 ) );
    dynamicsWorld->addRigidBody( groundBody );


    // Make animated compound objext.
    osg::MatrixTransform* animObj = createAnimatedObject( osg::Vec3( .5, .5, .5 ) );
    osg::AnimationPathCallback * apc = new osg::AnimationPathCallback(
        createAnimationPath( osg::Vec3( -6, -10, -9 ),
                             osg::Vec3( 16, 19, 0 ), 5 ),
        0, 1 );
    animObj->setUpdateCallback( apc );
    root->addChild( animObj );

    //   Add animated object to Bullet
    btRigidBody * animBody = createBTCompound( animObj, osg::Vec3( -9, -3, -9 ) );
    animBody->setCollisionFlags( animBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
    animBody->setActivationState( DISABLE_DEACTIVATION );
    dynamicsWorld->addRigidBody( animBody );

    //   Add osgBullet code to link OSG and Bullet representations.
    osgbCollision::RefBulletObject< btRigidBody >* animRB =
        new osgbCollision::RefBulletObject< btRigidBody >( animBody );
    animObj->setUserData( animRB );

    osgbDynamics::RigidBodyAnimation * rba = new osgbDynamics::RigidBodyAnimation;
    apc->setNestedCallback( rba );


    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 10, 30, 800, 600 );
    osgGA::TrackballManipulator * tb = new osgGA::TrackballManipulator();
    tb->setHomePosition( osg::Vec3( 20, -24, 3 ),
                        osg::Vec3( 2, 0, -10 ),
                        osg::Vec3( 0, 0, 1 ) );
    viewer.setCameraManipulator( tb );
    viewer.setSceneData( root.get() );

    double currSimTime;
    double prevSimTime = viewer.getFrameStamp()->getSimulationTime();

    viewer.realize();

    while( !viewer.done() )
    {
        currSimTime = viewer.getFrameStamp()->getSimulationTime();
        dynamicsWorld->stepSimulation( currSimTime - prevSimTime );
        prevSimTime = currSimTime;
        viewer.frame();
    }

    return( 0 );
}
