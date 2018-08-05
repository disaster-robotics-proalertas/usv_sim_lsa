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

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osg/Switch>
#include <osg/Geode>
#include <osg/Geometry>

#include <osgwTools/AbsoluteModelTransform.h>
#include <osgwTools/InsertRemove.h>
#include <osgwTools/Shapes.h>
#include <osgwTools/Version.h>

#include <osgbDynamics/RigidBody.h>
#include <osgbDynamics/MotionState.h>
#include <osgbCollision/Utils.h>
#include <osgbCollision/GLDebugDrawer.h>

#include <btBulletDynamicsCommon.h>

#include <osgwTools/FindNamedNode.h>

#include <vector>
#include <string>
#include <osg/io_utils>


const bool showSimple( false );
const bool showOrig( true );
const bool showCube( false );
const bool showTruck( false );
const bool enableSimple( showSimple && true );
const bool enableOrig( showOrig && true );
const bool enableCube( showCube && true );
const bool enableTruck( showTruck && true );

void enablePhysics( osg::Node* root, const std::string& nodeName, btDynamicsWorld* bw, const osg::Vec3& com=osg::Vec3( 0., 0., 0. ) );



osg::Node*
makeScene( btDynamicsWorld* bw )
{
    osg::Switch* root = new osg::Switch;


    // Make the ground plane
    {
        btBoxShape* shape = new btBoxShape( btVector3( 40., 40., .5 ) );

		btRigidBody::btRigidBodyConstructionInfo rbInfo( 0., NULL, shape );
		btRigidBody* body = new btRigidBody( rbInfo );
        body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
        body->setActivationState( DISABLE_DEACTIVATION );

		bw->addRigidBody(body);
	}

    // Make wireframe reference grids to aid in visual verification of location.
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( osgwTools::makeWirePlane( osg::Vec3( -10, -10, 10 ),
        osg::Vec3( 20, 0, 0 ), osg::Vec3( 0, 20, 0 ), osg::Vec2s( 4, 4 ) ) );
    geode->addDrawable( osgwTools::makeWirePlane( osg::Vec3( -10, 0, 0 ),
        osg::Vec3( 20, 0, 0 ), osg::Vec3( 0, 0, 20 ), osg::Vec2s( 4, 4 ) ) );
    root->addChild( geode );


	std::string fileName;
	fileName = "cow.osg";
    osg::ref_ptr< osg::Node > cowNode = osgDB::readNodeFile( fileName );
	if( !cowNode.valid() )
	{
		osg::notify( osg::FATAL ) << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH includes the OSG sample data directory." << std::endl;
		exit( 0 );
	}
	fileName = "dumptruck.osg";
    osg::ref_ptr< osg::Node > truckNode = osgDB::readNodeFile( fileName );
	if( !truckNode.valid() )
	{
		osg::notify( osg::FATAL ) << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH includes the OSG sample data directory." << std::endl;
		exit( 0 );
	}
	fileName = "offcube.osg";
    osg::ref_ptr< osg::Node > cubeNode = osgDB::readNodeFile( fileName );
	if( !cubeNode.valid() )
	{
		osg::notify( osg::FATAL ) << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH includes the osgBullet data directory." << std::endl;
		exit( 0 );
	}
	fileName = "block.osg";
    osg::ref_ptr< osg::Node > blockNode = osgDB::readNodeFile( fileName );
	if( !blockNode.valid() )
	{
		osg::notify( osg::FATAL ) << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH includes the osgBullet data directory." << std::endl;
		exit( 0 );
	}


    osg::MatrixTransform* mt0;
    osg::MatrixTransform* mt1;
    osg::MatrixTransform* mt2;
    osg::Group* grp;


    osg::Group* simpleGrp = new osg::Group;

    // Cube, local scale, off-origin COM
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( 0, 0, 9 ) ) );
    grp = new osg::Group;
    grp->setName( "offcube0" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    simpleGrp->addChild( mt0 );
    mt0->addChild( grp );
    grp->addChild( cubeNode.get() );

    const unsigned int simpleGrpIdx( root->getNumChildren() );
    root->addChild( simpleGrp );


    osg::Group* origGrp = new osg::Group;

    // Cow, not scaled, off-origin COM.
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( -7, 7, 13 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( 2., osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( 1., 1., 1. ) ) );
    grp = new osg::Group;
    grp->setName( "cow0" );
    origGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( cowNode.get() );

    // Truck, local scale, off-origin COM
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( 7, 7, 13 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( .9, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( .3, .3, .3 ) ) );
    grp = new osg::Group;
    grp->setName( "truck0" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    origGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( truckNode.get() );

    // Cube, local scale, off-origin COM
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( 0, 8, 9 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( -.6, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( 3.5, 3.5, 3.5 ) ) );
    grp = new osg::Group;
    grp->setName( "offcube0" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    origGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( cubeNode.get() );

    // Truck, global scale, off-origin COM
    mt0 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( .25, .25, .25 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( -30, -21, 39 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::rotate( -.9, osg::Vec3( 0, 1, 0 ) ) );
    grp = new osg::Group;
    grp->setName( "truck1" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    origGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( truckNode.get() );

    // Block, local scale, origin COM
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( 9, -7, 13 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( .6, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( .5, .5, .5 ) ) );
    grp = new osg::Group;
    grp->setName( "block0" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    origGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( blockNode.get() );

    // Block test case
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( 5, 0, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( 0., osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( .5, .5, .5 ) ) );
    grp = new osg::Group;
    grp->setName( "block1" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    origGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( blockNode.get() );

    const unsigned int origGrpIdx( root->getNumChildren() );
    root->addChild( origGrp );


    osg::Group* cubeGrp = new osg::Group;
    const float cubeScale( 2.f );

    // Cube, local scale, off-origin COM
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( -10, -10, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( .1, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( cubeScale, cubeScale, cubeScale ) ) );
    grp = new osg::Group;
    grp->setName( "offcube1" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    cubeGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( cubeNode.get() );

    // Cube, local scale, off-origin COM
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( -5, -10, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( .2, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( cubeScale, cubeScale, cubeScale ) ) );
    grp = new osg::Group;
    grp->setName( "offcube2" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    cubeGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( cubeNode.get() );

    // Cube, local scale, off-origin COM
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( 0, -10, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( .3, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( cubeScale, cubeScale, cubeScale ) ) );
    grp = new osg::Group;
    grp->setName( "offcube3" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    cubeGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( cubeNode.get() );

    // Cube, local scale, off-origin COM
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( 5, -10, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( .4, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( cubeScale, cubeScale, cubeScale ) ) );
    grp = new osg::Group;
    grp->setName( "offcube4" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    cubeGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( cubeNode.get() );

    // Cube, local scale, off-origin COM
    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( 10, -10, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( .5, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( cubeScale, cubeScale, cubeScale ) ) );
    grp = new osg::Group;
    grp->setName( "offcube5" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    cubeGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( cubeNode.get() );

    const unsigned int cubeGrpIdx( root->getNumChildren() );
    root->addChild( cubeGrp );


    osg::Group* truckGrp = new osg::Group;
    float sFac( .25 );

    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( -18, 0, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( osg::PI/2., osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( sFac, sFac, sFac ) ) );
    grp = new osg::Group;
    grp->setName( "truck2" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    truckGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( truckNode.get() );

    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( -9, 0, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( osg::PI, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( sFac, sFac, sFac ) ) );
    grp = new osg::Group;
    grp->setName( "truck3" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    truckGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( truckNode.get() );

    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( 0, 0, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( osg::PI*1.5, osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( sFac, sFac, sFac ) ) );
    grp = new osg::Group;
    grp->setName( "truck4" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    truckGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( truckNode.get() );

    mt0 = new osg::MatrixTransform(
        osg::Matrix::translate( osg::Vec3( 9, 0, 10 ) ) );
    mt1 = new osg::MatrixTransform(
        osg::Matrix::rotate( osg::PI*2., osg::Vec3( 0, 1, 0 ) ) );
    mt2 = new osg::MatrixTransform(
        osg::Matrix::scale( osg::Vec3( sFac, sFac, sFac ) ) );
    grp = new osg::Group;
    grp->setName( "truck5" );
    grp->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
    truckGrp->addChild( mt0 );
    mt0->addChild( mt1 );
    mt1->addChild( mt2 );
    mt2->addChild( grp );
    grp->addChild( truckNode.get() );

    const unsigned int truckGrpIdx( root->getNumChildren() );
    root->addChild( truckGrp );


    root->setValue( simpleGrpIdx, showSimple );
    root->setValue( origGrpIdx, showOrig );
    root->setValue( cubeGrpIdx, showCube );
    root->setValue( truckGrpIdx, showTruck );

    if( enableSimple )
    {
        enablePhysics( root, "offcube0", bw );
    }

    if( enableOrig )
    {
        enablePhysics( root, "cow0", bw );
        enablePhysics( root, "truck0", bw );
        enablePhysics( root, "offcube0", bw );
        enablePhysics( root, "truck1", bw );
        enablePhysics( root, "block0", bw );
        enablePhysics( root, "block1", bw, osg::Vec3( 3., 0., 0. ) );
    }

    if( enableCube )
    {
        enablePhysics( root, "offcube1", bw );
        enablePhysics( root, "offcube2", bw );
        enablePhysics( root, "offcube3", bw );
        enablePhysics( root, "offcube4", bw );
        enablePhysics( root, "offcube5", bw );
    }

    if( enableTruck )
    {
        enablePhysics( root, "truck2", bw );
        enablePhysics( root, "truck3", bw );
        enablePhysics( root, "truck4", bw );
        enablePhysics( root, "truck5", bw );
    }


    return( (osg::Node*) root );
}



btDynamicsWorld* initPhysics()
{
    btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    dynamicsWorld->setGravity( btVector3( 0, 0, -9.8 ) );

    return( dynamicsWorld );
}


osg::Vec3
findScale( osg::NodePath& np )
{
    // Note this code is doing a top-down search rather than a bottom up search.
    // Really this is of no consequence; either way, the code stops as soon as it
    // finds a scale, so it only supports a single scale in the NodePath and can
    // therefore search in either direction.
    osg::NodePath::const_iterator it;
    for( it=np.begin(); it != np.end(); it++ )
    {
        osg::Node* node( *it );
        osg::Transform* trans( node->asTransform() );
        if( trans == NULL )
            continue;
        osg::MatrixTransform* mt = dynamic_cast< osg::MatrixTransform* >( trans );
        osg::PositionAttitudeTransform* pat = dynamic_cast< osg::PositionAttitudeTransform* >( trans );
        if( mt != NULL )
        {
            const osg::Matrix& m( mt->getMatrix() );
            osg::Vec3 sVec = m.getScale();
            if( sVec != osg::Vec3( 1., 1., 1. ) )
                return( sVec );
        }
        else if( pat != NULL )
        {
            osg::Vec3 sVec = pat->getScale();
            if( sVec != osg::Vec3( 1., 1., 1. ) )
                return( sVec );
        }
    }
    return( osg::Vec3( 1., 1., 1. ) );
}

// Find the Node with name nodeName in the scene graph rooted at root.
// Enable physics for that node using the bullet world bw. If the COM
// is anything other than (0,0,0), use it.
void
enablePhysics( osg::Node* root, const std::string& nodeName, btDynamicsWorld* bw, const osg::Vec3& com )
{
    bool useCom( com != osg::Vec3( 0., 0., 0. ) );

    osgwTools::FindNamedNode fnn( nodeName );
    root->accept( fnn );
    if( fnn._napl.empty() )
    {
        osg::notify( osg::WARN ) << "Can't find node \"" << nodeName << "\"" << std::endl;
        return;
    }
    osg::Node* node = fnn._napl[ 0 ].first;
    osg::NodePath np = fnn._napl[ 0 ].second;
    osg::BoundingSphere bs( node->getBound() );
    const osg::Matrix parentTrans = osg::computeLocalToWorld( np ); // Note that this might contain a scale.

    // Find out if we are scaled, and if so, what is the xyz scale vector.
    const osg::Vec3 sVec = findScale( np );

    osg::ref_ptr< osgwTools::AbsoluteModelTransform > amt = new osgwTools::AbsoluteModelTransform;
    amt->setDataVariance( osg::Object::DYNAMIC );
    osgwTools::insertAbove( node, amt.get() );


    osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = amt.get();
    cr->_shapeType = CONVEX_HULL_SHAPE_PROXYTYPE;
    if( useCom )
        cr->setCenterOfMass( com );
    cr->_mass = 1.;
    cr->_scale = sVec;
    cr->_parentTransform = parentTrans;
    btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );

    bw->addRigidBody( rb );
}


int
main( int argc,
      char ** argv )
{
    btDynamicsWorld* bulletWorld = initPhysics();
    osg::ref_ptr< osg::Group > root = new osg::Group;

    root->addChild( makeScene( bulletWorld ) );


    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 10, 30, 800, 600 );
    viewer.setSceneData( root.get() );
    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    tb->setHomePosition( osg::Vec3( 0., -70., 12. ), osg::Vec3( 0., 0., 12. ), osg::Vec3( 0., 0., 1. ) );
    viewer.setCameraManipulator( tb );


    osgbCollision::GLDebugDrawer dbgDraw;
    dbgDraw.setDebugMode( ~btIDebugDraw::DBG_DrawText );
    bulletWorld->setDebugDrawer( &dbgDraw );
    root->addChild( dbgDraw.getSceneGraph() );

    double currSimTime = viewer.getFrameStamp()->getSimulationTime();
    double prevSimTime = viewer.getFrameStamp()->getSimulationTime();
    viewer.realize();

    while( !viewer.done() )
    {
        dbgDraw.BeginDraw();

        currSimTime = viewer.getFrameStamp()->getSimulationTime();
        bulletWorld->stepSimulation( currSimTime - prevSimTime );
        prevSimTime = currSimTime;

        bulletWorld->debugDrawWorld();
        dbgDraw.EndDraw();

        viewer.frame();
    }

    return( 0 );
}

