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

#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgGA/TrackballManipulator>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/AnimationPath>
#include <osg/PolygonMode>
#include <osg/PolygonOffset>

#include <btBulletDynamicsCommon.h>

#include <osg/io_utils>
#include <iostream>

#include <osgbDynamics/MotionState.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbDynamics/RigidBodyAnimation.h>

osg::AnimationPath * createAnimationPath( const osg::Vec3 & center,
                                          float radius,
                                          double looptime )
{
    /* set up the animation path */
    osg::AnimationPath * animationPath = new osg::AnimationPath;

    animationPath->setLoopMode( osg::AnimationPath::LOOP );

    int numSamples = 40;
    float yaw = 0.0f;
    float yaw_delta = 2.0f * osg::PI / ( ( float )numSamples - 1.0f );
    float roll = osg::inDegrees( 30.0f );

    double time = 0.0f;
    double time_delta = looptime / ( double )numSamples;
    for( int i = 0; i < numSamples; ++i )
    {
        osg::Vec3 position( center + osg::Vec3( sinf( yaw ) * radius, cosf( yaw ) * radius, 0.0f ) );
        osg::Quat rotation( osg::Quat( roll, osg::Vec3( 0.0, 1.0, 0.0 ) ) * osg::Quat( -( yaw + osg::inDegrees( 90.0f ) ), osg::Vec3( 0.0, 0.0, 1.0 ) ) );

        animationPath->insert( time, osg::AnimationPath::ControlPoint( position, rotation ) );

        yaw += yaw_delta;
        time += time_delta;
    }
    return( animationPath );
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

btRigidBody * createBTBox( osg::MatrixTransform * box,
                          osg::Vec3 center )
{
    btCollisionShape * collision = osgbCollision::btBoxCollisionShapeFromOSG( box );

    osgbDynamics::MotionState * motion = new osgbDynamics::MotionState();
    motion->setTransform( box );
    motion->setParentTransform( osg::Matrix::translate( center ) );

    btScalar mass( 0.0 );
    btVector3 inertia( 0, 0, 0 );
    btRigidBody::btRigidBodyConstructionInfo rb( mass, motion, collision, inertia );
    btRigidBody * body = new btRigidBody( rb );

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

float randomFloatInRange( std::pair< float, float > range =
                            std::pair< float, float >( -10, 10 ) )
{
    return( range.first + (range.second-range.first)*rand()/static_cast< float >( RAND_MAX ) );
}

btVector3 randomBVec3InRange( std::pair< btVector3, btVector3 > range =
                        std::pair< btVector3, btVector3 >(
                            btVector3( -1, -1, -1),
                            btVector3(  1,  1,  1) ) )
{
    return(
        btVector3( 
            randomFloatInRange(
                std::pair< float, float >( range.first[0], range.second[0] ) ),
            randomFloatInRange(                                                
                std::pair< float, float >( range.first[1], range.second[1] ) ),
            randomFloatInRange(                                                
                std::pair< float, float >( range.first[2], range.second[2] ) ) ) );
}


/* \cond */
class GliderUpdateCallback : public osg::NodeCallback
{
public:
    GliderUpdateCallback( btRigidBody * body )
        : body_( body )
        , basetime_( 0.0 )
        {}
    virtual void operator()( osg::Node* node, osg::NodeVisitor* nv )
    {
        double now = nv->getFrameStamp()->getSimulationTime();
        if ( ( now - basetime_ ) > 5.5 )
        {
            basetime_ = now;
            btVector3 punch = randomBVec3InRange(
                std::pair< btVector3, btVector3 >(
                btVector3( -10, -10, -.5 ), btVector3(  10,  10, .5 ) ) );
            osg::notify( osg::NOTICE ) << "punch!"
                << punch[0] << " "
                << punch[1] << " "
                << punch[2] << std::endl;
            body_->setLinearVelocity( punch );
            body_->setAngularVelocity( randomBVec3InRange() );
        }
        traverse( node, nv );
    }
private:
    btRigidBody *   body_;
    double          basetime_;
};
/* \endcond */


osg::MatrixTransform * createModel( btDynamicsWorld * dynamicsWorld )
{
/*
 * BEGIN: Create physics object code.
 *  OSG CODE
 */
    osg::ref_ptr< osg::MatrixTransform > node;
	const std::string fileName( "glider.osg" );
    osg::ref_ptr< osg::Node > nodeDB = osgDB::readNodeFile( fileName );
	if( !nodeDB.valid() )
	{
		osg::notify( osg::FATAL ) << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH includes the OSG sample data directory." << std::endl;
		exit( 0 );
	}

    if( ( node = dynamic_cast< osg::MatrixTransform * >( nodeDB.get() ) ) == NULL )
    {
        node = new osg::MatrixTransform;
        node->addChild( nodeDB.get() );
    }

    /*  osgBullet code */
    osgbDynamics::MotionState * motion = new osgbDynamics::MotionState;
    motion->setTransform( node.get() );
    btCollisionShape * collision = osgbCollision::btConvexTriMeshCollisionShapeFromOSG( node.get() );
    // Create an OSG representation of the Bullet shape and attach it.
    // This is mainly for debugging.
    osg::Node* debugNode = osgbCollision::osgNodeFromBtCollisionShape( collision );
    node->addChild( debugNode );

    // Set debug node state.
    osg::StateSet* state = debugNode->getOrCreateStateSet();
    osg::PolygonMode* pm = new osg::PolygonMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
    state->setAttributeAndModes( pm );
    osg::PolygonOffset* po = new osg::PolygonOffset( -1, -1 );
    state->setAttributeAndModes( po );
    state->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    /*  BULLET CODE */
    btTransform bodyTransform;
    bodyTransform.setIdentity();
    bodyTransform.setOrigin( btVector3( 0, 0, 5 ) );
    motion->setWorldTransform( bodyTransform );

    btScalar mass( 1.0 );
    btVector3 inertia;
    collision->calculateLocalInertia( mass, inertia );
    btRigidBody::btRigidBodyConstructionInfo rbinfo( mass, motion, collision, inertia );
    btRigidBody * body = new btRigidBody( rbinfo );
    body->setLinearVelocity( btVector3( -5, -1, 0 ) );
    body->setAngularVelocity( btVector3( 1, 0, 0 ) );
    dynamicsWorld->addRigidBody( body );
    
    // kick thing around from time to time.
    node->setUpdateCallback( new GliderUpdateCallback( body ) );
    
    return( node.release() );
}

int main( int argc,
          char * argv[] )
{
    osg::ArgumentParser arguments( &argc, argv );
    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 10, 30, 800, 600 );

    osgGA::TrackballManipulator * tb = new osgGA::TrackballManipulator();

    tb->setHomePosition( osg::Vec3( 5, -12, 12 ),
                        osg::Vec3( -7, 0, -10 ),
                        osg::Vec3( 0, 0, 1 ) );
    viewer.setCameraManipulator( tb );

    osg::ref_ptr< osg::Group > root = new osg::Group;
    viewer.setSceneData( root.get() );

    osgDB::getDataFilePathList().push_back( "C:\\OpenSceneGraph\\Data" );

    btDynamicsWorld * dynamicsWorld = initPhysics();

    root->addChild( createModel( dynamicsWorld ) );

    /* BEGIN: Create environment boxes */
    osg::MatrixTransform * ground;
    btRigidBody * groundBody;

    float thin = .01;
    ground = createOSGBox( osg::Vec3( 10, 10, thin ) );
    root->addChild( ground );
    groundBody = createBTBox( ground, osg::Vec3( 0, 0, -10 ) );
    dynamicsWorld->addRigidBody( groundBody );

    ground = createOSGBox( osg::Vec3( 10, thin, 5 ) );
    root->addChild( ground );
    groundBody = createBTBox( ground, osg::Vec3( 0, 10, -5 ) );
    dynamicsWorld->addRigidBody( groundBody );

    ground = createOSGBox( osg::Vec3( 10, thin, 5 ) );
    root->addChild( ground );
    groundBody = createBTBox( ground, osg::Vec3( 0, -10, -5 ) );
    dynamicsWorld->addRigidBody( groundBody );

    ground = createOSGBox( osg::Vec3( thin, 10, 5 ) );
    root->addChild( ground );
    groundBody = createBTBox( ground, osg::Vec3( 10, 0, -5 ) );
    dynamicsWorld->addRigidBody( groundBody );

    ground = createOSGBox( osg::Vec3( thin, 10, 5 ) );
    root->addChild( ground );
    groundBody = createBTBox( ground, osg::Vec3( -10, 0, -5 ) );
    dynamicsWorld->addRigidBody( groundBody );
    /* END: Create environment boxes */

    /* BEGIN: Create animated box */
    /* OSG Code */
    osg::MatrixTransform * box = createOSGBox( osg::Vec3( .3, .3, .3 ) );
    osg::AnimationPathCallback * apc = new osg::AnimationPathCallback( createAnimationPath( osg::Vec3( 0, 0, -9.25 ), 9.4, 6 ), 0, 1 );
    box->setUpdateCallback( apc );
    root->addChild( box );

    /* Bullet Code */
    btRigidBody * boxBody = createBTBox( box, osg::Vec3( -9, -3, -9 ) );
    boxBody->setCollisionFlags( boxBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
    boxBody->setActivationState( DISABLE_DEACTIVATION );
    dynamicsWorld->addRigidBody( boxBody );

    /* osgBullet Code */
    osgbCollision::RefBulletObject< btRigidBody >* boxRigid =
        new osgbCollision::RefBulletObject< btRigidBody >( boxBody );
    box->setUserData( boxRigid );

    osgbDynamics::RigidBodyAnimation * rba = new osgbDynamics::RigidBodyAnimation;
    apc->setNestedCallback( rba );
    /* END: Create animated box */

    // bonus geometry
    root->addChild( osgDB::readNodeFiles( arguments ) );

    double currSimTime = viewer.getFrameStamp()->getSimulationTime();;
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

