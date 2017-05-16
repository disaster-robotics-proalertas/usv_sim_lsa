/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgBullet is (C) Copyright 2009 by Kenneth Mark Bryden
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

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgText/Text>

#include <osgbBullet/OSGToCollada.h>
#include <osgbBullet/MotionState.h>
#include <osgbBullet/CollisionShapes.h>
#include <osgbBullet/GLDebugDrawer.h>
#include <osgbBullet/Utils.h>

#include <btBulletDynamicsCommon.h>

#include <BulletMultiThreaded/SpuGatheringCollisionDispatcher.h>

#ifdef WIN32
#include "BulletMultiThreaded/Win32ThreadSupport.h"
#else
#include "BulletMultiThreaded/PosixThreadSupport.h"
#endif
#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#if( BT_BULLET_VERSION < 275 )
// Removed in v2.75
#include "BulletMultiThreaded/SpuParallelSolver.h"
#include "BulletMultiThreaded/SpuSolverTask/SpuParallellSolverTask.h"
#endif


//#define USE_BULLET_CUDA
#ifdef USE_BULLET_CUDA
#include "CUDA/btCudaBroadphase.h"
#endif

#include <sstream>
#include <osg/io_utils>
#include <string>
#include <map>


typedef enum {
    Serial,
    Parallel
} SerialParallelType;
SerialParallelType g_collDisp( Serial );

typedef enum {
    AxisSweep3,
    BigAxisSweep3,
#if( BT_BULLET_VERSION < 276 )
    // Removed in Bullet v2.76.
    MultiSaP,
#endif
#ifdef USE_BULLET_CUDA
    Cuda,
#endif
    Dbvt
} BroadphaseType;
#ifdef USE_BULLET_CUDA
BroadphaseType g_broadphase( Cuda );
#else
BroadphaseType g_broadphase( Dbvt );
#endif

typedef enum {
    Hashed,
    Sorted,
    Default
} OverlappingPairCacheType;
OverlappingPairCacheType g_opc( Default );

SerialParallelType g_solver( Serial );


btBroadphaseInterface* broadphase;
btCollisionDispatcher* dispatcher;
btConstraintSolver* solver;

btDynamicsWorld*
initPhysics()
{
    //
    // Collision configuration, of type btCollisionConfiguration
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();


    //
    // Set up for multithreading, if necessary.
    unsigned int numDispatchThreads( 2 );
    unsigned int numSolverThreads( 2 );

    osg::notify( osg::ALWAYS ) << std::endl;


    //
    // Collision dispatcher, of type btDispatcher.
    dispatcher = NULL;
    if( g_collDisp == Parallel )
    {
        osg::notify( osg::ALWAYS ) << "Dispatch: Parallel (SpuGatheringCollisionDispatcher)" << std::endl;

        btThreadSupportInterface* threadSupport( NULL );
#ifdef WIN32
        Win32ThreadSupport::Win32ThreadConstructionInfo ci(
								    "collision",
								    processCollisionTask,
								    createCollisionLocalStoreMemory,
								    numDispatchThreads );
        threadSupport = new Win32ThreadSupport( ci );
#else
        PosixThreadSupport::ThreadConstructionInfo ci(
								    "collision",
								    processCollisionTask,
								    createCollisionLocalStoreMemory,
								    numDispatchThreads );
        threadSupport = new PosixThreadSupport( ci );
#endif
        dispatcher = new SpuGatheringCollisionDispatcher( threadSupport, numDispatchThreads, collisionConfiguration );
    }
    else if( g_collDisp == Serial )
    {
        osg::notify( osg::ALWAYS ) << "Dispatch: Serial (btCollisionDispatcher)" << std::endl;
        dispatcher = new btCollisionDispatcher( collisionConfiguration );
    }
    else
        osg::notify( osg::FATAL ) << "Unknown dispatcher type." << std::endl;


    //
    // Overlapping pair cache used by the broadphase interface
    btOverlappingPairCache* opc( NULL );
    if( g_opc == Hashed )
    {
        osg::notify( osg::ALWAYS ) << "Pair cache: btHashedOverlappingPairCache" << std::endl;
        opc = new btHashedOverlappingPairCache();
    }
    else if( g_opc == Sorted )
    {
        osg::notify( osg::ALWAYS ) << "Pair cache: btSortedOverlappingPairCache" << std::endl;
	    opc = new btSortedOverlappingPairCache();
    }
    else
        osg::notify( osg::ALWAYS ) << "Pair cache: Default (hashed)" << std::endl;


    //
    // Broadphase interface, of type btBroadphaseInterface.
    broadphase = NULL;
    btVector3 worldMin( -100, -100, -100 );
    btVector3 worldMax( 100, 100, 100 );
    unsigned short maxHandles( 16535 );
    if( g_broadphase == AxisSweep3 )
    {
        osg::notify( osg::ALWAYS ) << "Broadphase: btAxisSweep3" << std::endl;
        broadphase = new btAxisSweep3( worldMin, worldMax, maxHandles, opc );
    }
    else if( g_broadphase == BigAxisSweep3 )
    {
        osg::notify( osg::ALWAYS ) << "Broadphase: bt32BitAxisSweep3" << std::endl;
        broadphase = new bt32BitAxisSweep3( worldMin, worldMax, maxHandles, opc );
    }
#if( BT_BULLET_VERSION < 276 )
    else if( g_broadphase == MultiSaP )
    {
        osg::notify( osg::ALWAYS ) << "Broadphase: btMultiSapBroadphase" << std::endl;
        int maxProxies( 16384 );
        broadphase = new btMultiSapBroadphase( maxProxies, opc );
    }
#endif
#ifdef USE_BULLET_CUDA
    else if( g_broadphase == Cuda )
    {
        osg::notify( osg::ALWAYS ) << "Broadphase: Cuda bt3DGridBroadphase" << std::endl;

        int sizex( 64 ), sizey( 64 ), sizez( 64 );
        int maxSmallProxies( 512 );
        int maxLargeProxies( 256 );
        int maxPairsPerBody( 64 );
        broadphase = new bt3DGridBroadphase( worldMin, worldMax,
					   sizex, sizey, sizez, 
					   maxSmallProxies, maxLargeProxies, maxPairsPerBody );
    }
#endif
    else if( g_broadphase == Dbvt )
    {
        osg::notify( osg::ALWAYS ) << "Broadphase: btDbvtBroadphase" << std::endl;
        broadphase = new btDbvtBroadphase( opc );
    }
    else
        osg::notify( osg::FATAL ) << "Unknown broadphase type." << std::endl;


    //
    // Constraint solver, of type btConstraintSolver
    solver = NULL;
#if( BT_BULLET_VERSION < 275 )
    if( g_solver == Parallel )
    {
        osg::notify( osg::ALWAYS ) << "Solver: Parallel (btParallelSequentialImpulseSolver)" << std::endl;

        btThreadSupportInterface* threadSupport( NULL );
#ifdef WIN32
        Win32ThreadSupport::Win32ThreadConstructionInfo ci(
								    "solver",
								    processSolverTask,
								    createSolverLocalStoreMemory,
								    numSolverThreads );
        threadSupport = new Win32ThreadSupport( ci );
#else
        PosixThreadSupport::ThreadConstructionInfo ci(
								    "solver",
								    processSolverTask,
								    createSolverLocalStoreMemory,
								    numSolverThreads );
        threadSupport = new PosixThreadSupport( ci );
#endif
        solver = new btParallelSequentialImpulseSolver( threadSupport, numSolverThreads );
    }
    else
#endif
         if( g_solver == Serial )
    {
        osg::notify( osg::ALWAYS ) << "Solver: Serial (btSequentialImpulseConstraintSolver)" << std::endl;
        solver = new btSequentialImpulseConstraintSolver;
    }
    else
        osg::notify( osg::FATAL ) << "Unknown solver type." << std::endl;


    //
    // Discrete (non-continuous) is the only supported dynamics world object at this time (Bullet 2.74).
    btDiscreteDynamicsWorld* dw = new btDiscreteDynamicsWorld( dispatcher, broadphase, solver, collisionConfiguration );

	dw->getSolverInfo().m_numIterations = 4;
	dw->getSolverInfo().m_friction = 500000.;
    dw->getSolverInfo().m_damping = 32.;
	dw->getSolverInfo().m_solverMode = SOLVER_SIMD+SOLVER_USE_WARMSTARTING;

	dw->getDispatchInfo().m_enableSPU = true;
    dw->setGravity( btVector3( 0, 0, -9.8 ) );


    osg::notify( osg::ALWAYS ) << std::endl;

    return( dw );
}


/* \cond */
class InteractionManipulator : public osgGA::GUIEventHandler
{
public:
    InteractionManipulator( btDynamicsWorld* world, osg::Group* sg )
      : _world( world ),
        _sg( sg )
    {}

    void setInitialTransform( btRigidBody* rb, osg::Matrix m )
    {
        _posMap[ rb ] = m;
    }

    void updateView( osg::Camera* camera )
    {
        osg::Vec3 center, up;
        camera->getViewMatrixAsLookAt( _viewPos, center, up );
        _viewDir = center - _viewPos;
    }

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& )
    {
        switch( ea.getEventType() )
        {
            case osgGA::GUIEventAdapter::KEYUP:
            {
                if (ea.getKey()==osgGA::GUIEventAdapter::KEY_BackSpace)
                {
                    reset();
                    return true;
                }
                if (ea.getKey()==osgGA::GUIEventAdapter::KEY_Return)
                {
                    fire();
                    return true;
                }

                return false;
            }

            default:
            break;
        }
        return false;
    }

protected:
    btDynamicsWorld* _world;
    osg::ref_ptr< osg::Group > _sg;

    osg::Vec3 _viewPos, _viewDir;

    typedef std::map< btRigidBody*, osg::Matrix > PosMap;
    PosMap _posMap;

    typedef std::list< osg::ref_ptr< osg::Node > > NodeList;
    NodeList _nodeList;

    void reset()
    {
        PosMap::iterator it;
        for( it=_posMap.begin(); it!=_posMap.end(); it++ )
        {
            btRigidBody* rb = it->first;
            btTransform t = osgbBullet::asBtTransform( it->second );
            rb->setWorldTransform( t );
        }
    }

    void fire()
    {
        osg::Sphere* sp = new osg::Sphere( osg::Vec3( 0., 0., 0. ), .5 );
        osg::ShapeDrawable* shape = new osg::ShapeDrawable( sp );
        osg::Geode* geode = new osg::Geode();
        geode->addDrawable( shape );
        osg::ref_ptr< osgwTools::AbsoluteModelTransform > amt = new osgwTools::AbsoluteModelTransform;
        amt->addChild( geode );
        _sg->addChild( amt.get() );

        btSphereShape* collision = new btSphereShape( .5 );

        osgbBullet::MotionState* motion = new osgbBullet::MotionState;
        motion->setTransform( amt.get() );

        motion->setParentTransform( osg::Matrix::translate( _viewPos ) );

        btScalar mass( 1. );
        btVector3 inertia( btVector3( 0., 0., 0. ) );//osgbBullet::asBtVector3( _viewDir ) );
        collision->calculateLocalInertia( mass, inertia );
        btRigidBody::btRigidBodyConstructionInfo rbinfo( mass, motion, collision, inertia );
        btRigidBody* body = new btRigidBody( rbinfo );
        body->setLinearVelocity( osgbBullet::asBtVector3( _viewDir * 50. ) );
        _world->addRigidBody( body );
    }
};
/* \endcond */


osg::ref_ptr< osg::Node > modelNode( NULL );

osg::MatrixTransform*
makeModel( const std::string& fileName, btDynamicsWorld* bw, osg::Vec3 pos, InteractionManipulator* im )
{
    osg::Matrix m( osg::Matrix::translate( pos ) );
    osg::MatrixTransform* root = new osg::MatrixTransform();// m );
    osgwTools::AbsoluteModelTransform* amt = new osgwTools::AbsoluteModelTransform;
    amt->setDataVariance( osg::Object::DYNAMIC );
    root->addChild( amt );

    if( !modelNode.valid() )
	{
        modelNode = osgDB::readNodeFile( fileName );
		if( !modelNode.valid() )
		{
			osg::notify( osg::FATAL ) << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH is set correctly." << std::endl;
			exit( 0 );
		}
	}
    amt->addChild( modelNode.get() );

    osgbBullet::OSGToCollada converter;
    converter.setSceneGraph( modelNode.get() );
    converter.setShapeType( BOX_SHAPE_PROXYTYPE );
    converter.setMass( .2 );
    converter.setOverall( false );
    converter.convert();

    btRigidBody* rb = converter.getRigidBody();
    osgbBullet::MotionState* motion = new osgbBullet::MotionState;
    motion->setTransform( amt );
    motion->setParentTransform( m );
    rb->setMotionState( motion );
    btVector3 inertia( 0., 0., 0. );
    rb->getCollisionShape()->calculateLocalInertia( 1., inertia );

    rb->setActivationState( DISABLE_DEACTIVATION );
    im->setInitialTransform( rb, m );
    bw->addRigidBody( rb );

    return( root );
}

osg::MatrixTransform*
makeCow( btDynamicsWorld* bw, osg::Vec3 pos, InteractionManipulator* im )
{
    osg::Matrix m( osg::Matrix::rotate( 1.5, osg::Vec3( 0., 0., 1. ) ) *
        osg::Matrix::translate( pos ) );
    osg::MatrixTransform* root = new osg::MatrixTransform( m );
    osgwTools::AbsoluteModelTransform* amt = new osgwTools::AbsoluteModelTransform;
    amt->setDataVariance( osg::Object::DYNAMIC );
    root->addChild( amt );

	const std::string fileName( "cow.osg" );
    osg::Node* node = osgDB::readNodeFile( fileName );
	if( node == NULL )
	{
		osg::notify( osg::FATAL ) << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH includes the OSG sample data directory." << std::endl;
		exit( 0 );
	}
    amt->addChild( node );

    btCollisionShape* cs = osgbBullet::btConvexTriMeshCollisionShapeFromOSG( node );
    osgbBullet::MotionState* motion = new osgbBullet::MotionState();
    motion->setTransform( amt );
    motion->setParentTransform( m );
    btScalar mass( 2. );
    btVector3 inertia( 0, 0, 0 );
    cs->calculateLocalInertia( mass, inertia );
    btRigidBody::btRigidBodyConstructionInfo rb( mass, motion, cs, inertia );
    btRigidBody* body = new btRigidBody( rb );
    body->setActivationState( DISABLE_DEACTIVATION );
    im->setInitialTransform( body, m );
    bw->addRigidBody( body );

    return( root );
}


osg::ref_ptr< osgText::Text > frameRateText;
osg::ref_ptr< osgText::Text > numObjectsText;
osg::ref_ptr< osgText::Text > pairsText;
osg::ref_ptr< osgText::Text > contactsText;

osg::Camera*
createHud( float w, float h )
{
    osg::ref_ptr< osg::Camera > hudCam = new osg::Camera;
    hudCam->setClearMask( GL_DEPTH_BUFFER_BIT );
    hudCam->setRenderOrder( osg::Camera::POST_RENDER );
    hudCam->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
    hudCam->setProjectionMatrix( osg::Matrix::ortho( 0., (float)w, 0., (float)h, -1., 1. ) );
    hudCam->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    osg::ref_ptr< osgText::Text > textClone = new osgText::Text;

    float charPixels( 23.f );
    float linePixels( 27.f );
    float col0( 5.f );
    float col1( 230.f );
    textClone->setFont( "arial.ttf" );
    textClone->setText( "uninitialized" );
    textClone->setAxisAlignment( osgText::TextBase::XY_PLANE );
    textClone->setColor( osg::Vec4( 1., 1., 1., 1. ) );
    textClone->setCharacterSize( charPixels );

    osg::Geode* geode = new osg::Geode;
    hudCam->addChild( geode );

    osg::ref_ptr< osgText::Text > frameRateLabel = new osgText::Text( *textClone );
    frameRateLabel->setText( "Simulation time" );
    frameRateLabel->setPosition( osg::Vec3( col0, (float)h - linePixels, 0. ) );
    geode->addDrawable( frameRateLabel.get() );

    frameRateText = new osgText::Text( *textClone );
    frameRateText->setDataVariance( osg::Object::DYNAMIC );
    frameRateText->setPosition( osg::Vec3( col1, (float)h - linePixels, 0. ) );
    geode->addDrawable( frameRateText.get() );


    osg::ref_ptr< osgText::Text > numObjectsLabel = new osgText::Text( *textClone );
    numObjectsLabel->setText( "# coll objs" );
    numObjectsLabel->setPosition( osg::Vec3( col0, (float)h - (linePixels * 2.f), 0. ) );
    geode->addDrawable( numObjectsLabel.get() );

    numObjectsText = new osgText::Text( *textClone );
    numObjectsText->setDataVariance( osg::Object::DYNAMIC );
    numObjectsText->setPosition( osg::Vec3( col1, (float)h - (linePixels * 2.f), 0. ) );
    geode->addDrawable( numObjectsText.get() );


    osg::ref_ptr< osgText::Text > pairsLabel = new osgText::Text( *textClone );
    pairsLabel->setText( "# overlap pairs" );
    pairsLabel->setPosition( osg::Vec3( col0, (float)h - (linePixels * 3.f), 0. ) );
    geode->addDrawable( pairsLabel.get() );

    pairsText = new osgText::Text( *textClone );
    pairsText->setDataVariance( osg::Object::DYNAMIC );
    pairsText->setPosition( osg::Vec3( col1, (float)h - (linePixels * 3.f), 0. ) );
    geode->addDrawable( pairsText.get() );


    osg::ref_ptr< osgText::Text > contactsLabel = new osgText::Text( *textClone );
    contactsLabel->setText( "# contact pts" );
    contactsLabel->setPosition( osg::Vec3( col0, (float)h - (linePixels * 4.f), 0. ) );
    geode->addDrawable( contactsLabel.get() );

    contactsText = new osgText::Text( *textClone );
    contactsText->setDataVariance( osg::Object::DYNAMIC );
    contactsText->setPosition( osg::Vec3( col1, (float)h - (linePixels * 4.f), 0. ) );
    geode->addDrawable( contactsText.get() );


    return( hudCam.release() );
}

int
main( int argc,
      char ** argv )
{
    btDynamicsWorld* bulletWorld = initPhysics();
    osg::Group* root = new osg::Group;

    InteractionManipulator* im = new InteractionManipulator( bulletWorld, root );

    std::string fileName( "dice.osg" );
    if( argc > 1 )
        // Seconf param is file name.
        fileName = std::string( argv[ 1 ] );

    // Make dice pyramid.
    int xCount( 7 );
    int yCount( 7 );
    float xStart( -4. );
    float yStart( -3. );
    const float zInc( 2.5 );
    float z( 1.75 );
    while( xCount && yCount )
    {
        float x, y;
        int xIdx, yIdx;
        for( y=yStart, yIdx=0; yIdx<yCount; y+=2.25, yIdx++ )
        {
            for( x=xStart, xIdx=0; xIdx<xCount; x+=2.25, xIdx++ )
            {
                osg::Vec3 pos( x, y, z );
                root->addChild( makeModel( fileName, bulletWorld, pos, im ) );
            }
        }
        xStart += 1.25;
        yStart += 1.25;
        xCount--;
        yCount--;
        z += zInc;
    }

    // Add a cow
    root->addChild( makeCow( bulletWorld, osg::Vec3( -11., 6., 4. ), im ) );

    // Make ground.
    {
        osg::Vec4 gp( 0, 0, 1, 0 );
        root->addChild( osgbBullet::generateGroundPlane( gp, bulletWorld ) );
    }

    int winW( 800 );
    int winH( 600 );
    root->addChild( createHud( (float)winW, (float)winH ) );



    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 50, 220, winW, winH );
    viewer.setSceneData( root );
    viewer.addEventHandler( im );

    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    tb->setHomePosition( osg::Vec3( 0., -26., 12. ), osg::Vec3( 0., 0., 2. ), osg::Vec3( 0., 0., 1. ) ); 
    viewer.setCameraManipulator( tb );

    viewer.addEventHandler( new osgViewer::StatsHandler );

    osg::Timer simTimer;
    viewer.realize();
    double prevSimTime = 0.;
    while( !viewer.done() )
    {
        const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
        double elapsed( currSimTime - prevSimTime );

        simTimer.setStartTick();
        bulletWorld->stepSimulation( elapsed );
        std::ostringstream ostr;
        ostr << simTimer.time_m();
        frameRateText->setText( ostr.str() );

        std::ostringstream ostr2;
        ostr2 << bulletWorld->getNumCollisionObjects();
        numObjectsText->setText( ostr2.str() );

        std::ostringstream ostr3;
        ostr3 << broadphase->getOverlappingPairCache()->getNumOverlappingPairs();
        pairsText->setText( ostr3.str() );

        btPersistentManifold** mp = dispatcher->getInternalManifoldPointer();
        int idx, totalContacts( 0 );
        for( idx=0; idx<dispatcher->getNumManifolds(); idx++ )
            totalContacts += mp[ idx ]->getNumContacts();;
        std::ostringstream ostr4;
        ostr4 << totalContacts;
        contactsText->setText( ostr4.str() );


        prevSimTime = currSimTime;
        viewer.frame();

        im->updateView( viewer.getCamera() );
    }

    return( 0 );
}
