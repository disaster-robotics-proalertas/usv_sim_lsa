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
#include <osgDB/FileUtils>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geode>

#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/CreationRecord.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbDynamics/GroundPlane.h>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgbCollision/Utils.h>

#include <osgbInteraction/SaveRestoreHandler.h>
#include <osgbInteraction/DragHandler.h>

#include <osgwTools/InsertRemove.h>
#include <osgwTools/FindNamedNode.h>

#include <btBulletDynamicsCommon.h>

#include <osg/io_utils>
#include <string>
#include <map>



btDiscreteDynamicsWorld* initPhysics()
{
    btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDiscreteDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    dynamicsWorld->setGravity( btVector3( 0, 0, -9.81 ) );

    return( dynamicsWorld );
}


int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );
    const bool debugDisplay( arguments.find( "--debug" ) > 0 );

    std::string restoreFileName;
    if( arguments.read( "--restore", restoreFileName ) )
    {
        if( osgDB::findDataFile( restoreFileName ).empty() )
        {
            osg::notify( osg::FATAL ) << "Can't find restore file: \"" << restoreFileName << "\"." << std::endl;
            return( 1 );
        }
    }

    btDiscreteDynamicsWorld* bulletWorld = initPhysics();
    osg::ref_ptr< osg::Group > root = new osg::Group;

    const std::string sceneFileName( "saverestore-scene.osg" );
    osg::Node* scene = osgDB::readNodeFile( sceneFileName );
    if( scene == NULL )
    {
        osg::notify( osg::FATAL ) << "saverestore: Can't load data file \"" << sceneFileName << "\"." << std::endl;
        return( 1 );
    }
    root->addChild( scene );

    osg::ref_ptr< osgbInteraction::SaveRestoreHandler > srh = new osgbInteraction::SaveRestoreHandler;

    // Restore the saved PhysicsState.
    if( !( restoreFileName.empty() ) )
        srh->restore( restoreFileName );

    // Find all nodes with names containing "-body". Turn each into a rigid body.
    osgwTools::FindNamedNode fnn( "-body" );
    fnn.setMatchMethod( osgwTools::FindNamedNode::CONTAINS );
    scene->accept( fnn );

    osgwTools::FindNamedNode::NodeAndPathList::const_iterator it;
    for( it = fnn._napl.begin(); it != fnn._napl.end(); it++ )
    {
        // Get the root node of the subgraph that we will make into a rigid body.
        // Also, get its NodePath and local to world transform.
        osg::Node* node = it->first;
        osg::NodePath np = it->second;
        osg::Matrix xform = osg::computeLocalToWorld( np );

        // Rigid bodies need to be rooted at an AbsoluteModelTransform.
        osgwTools::AbsoluteModelTransform* amt = new osgwTools::AbsoluteModelTransform;
        amt->setDataVariance( osg::Object::DYNAMIC );
        osgwTools::insertAbove( node, amt );

        // Manually insert the AMT above the node in the NodePath. Kind of ugly.
        np[ np.size() - 1 ] = amt;
        np.resize( np.size() + 1);
        np[ np.size() - 1 ] = node;

        osg::ref_ptr< osgbDynamics::CreationRecord > cr;
        osgbDynamics::PhysicsData* pd;
        if( restoreFileName.empty() )
        {
            // Not restoring. Configure the CreationRecord.
            cr = new osgbDynamics::CreationRecord;
            cr->_sceneGraph = amt;
            cr->_shapeType = CONVEX_HULL_SHAPE_PROXYTYPE;
            cr->setCenterOfMass( node->getBound().center() );
            cr->_parentTransform = xform;
            cr->_mass = 1.f;
            cr->_scale = xform.getScale();
            cr->_restitution = .5f;

            srh->add( it->first->getName(), cr.get() );
        }
        else
        {
            // Restoring.
            // Get the CreationRecord from the SaveRestoreHandler, which
            // contains our restored PhysicsData for this node.
            pd = srh->getPhysicsData( it->first->getName() );
            cr = pd->_cr;
            // _sceneGraph is a live address and can't be saved/restored, so set it manually.
            cr->_sceneGraph = amt;
        }

        btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );
        rb->setActivationState( DISABLE_DEACTIVATION );

        if( !( restoreFileName.empty() ) )
        {
            // Now that the rigid body has been created, set its saved transform and linear / angular velocities.
            rb->setWorldTransform( osgbCollision::asBtTransform( pd->_bodyWorldTransform ) );
            rb->setLinearVelocity( osgbCollision::asBtVector3( pd->_linearVelocity ) );
            rb->setAngularVelocity( osgbCollision::asBtVector3( pd->_angularVelocity ) );
        }

        // Required for DragHandler default behavior.
        amt->setUserData( new osgbCollision::RefRigidBody( rb ) );

        bulletWorld->addRigidBody( rb );

        srh->add( it->first->getName(), rb );
    }

    // Add ground
    const osg::Vec4 plane( 0., 0., 1., 0. );
    root->addChild( osgbDynamics::generateGroundPlane( plane, bulletWorld ) );


    osgbCollision::GLDebugDrawer* dbgDraw( NULL );
    if( debugDisplay )
    {
        dbgDraw = new osgbCollision::GLDebugDrawer();
        dbgDraw->setDebugMode( ~btIDebugDraw::DBG_DrawText );
        bulletWorld->setDebugDrawer( dbgDraw );
        root->addChild( dbgDraw->getSceneGraph() );
    }


    osgViewer::Viewer viewer( arguments );
    viewer.setUpViewInWindow( 30, 30, 768, 480 );
    viewer.setSceneData( root.get() );

    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    //tb->setHomePosition( osg::Vec3( 0., -8., 2. ), osg::Vec3( 0., 0., 1. ), osg::Vec3( 0., 0., 1. ) ); 
    viewer.setCameraManipulator( tb );
    viewer.getCamera()->setClearColor( osg::Vec4( .5, .5, .5, 1. ) );

    srh->capture();
    viewer.addEventHandler( srh.get() );

    osgViewer::Viewer::Cameras cams;
    viewer.getCameras( cams );
    osg::ref_ptr< osgbInteraction::DragHandler > dh =
        new osgbInteraction::DragHandler( bulletWorld, cams[ 0 ] );
    viewer.addEventHandler( dh.get() );

    viewer.realize();
    double prevSimTime = 0.;
    while( !viewer.done() )
    {
        if( dbgDraw != NULL )
            dbgDraw->BeginDraw();

        const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
        bulletWorld->stepSimulation( currSimTime - prevSimTime );
        prevSimTime = currSimTime;

        if( dbgDraw != NULL )
        {
            bulletWorld->debugDrawWorld();
            dbgDraw->EndDraw();
        }

        viewer.frame();
    }

    return( 0 );
}


/** \page saverestoreexample Save and Restore Example

Demonstrates saving and restoring osgBullet data to/from disk.

Use the --debug command line option to enable debug collision object display.

\section saverestorecontrols UI Controls

\li Delete: Reset the physics simulation to its initial state.
\li ctrl-leftmouse: Select and drag an object.

*/
