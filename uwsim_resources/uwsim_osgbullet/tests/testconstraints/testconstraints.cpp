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

#include "ctest.h"

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>

#include <osgbDynamics/Constraints.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbDynamics/GroundPlane.h>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgbCollision/Utils.h>
#include <osgbInteraction/DragHandler.h>
#include <osgbInteraction/SaveRestoreHandler.h>

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

    int ctestPos = arguments.find( "--ctest" );
    if( ctestPos > 0 )
    {
        if( argc <= ( ctestPos+1 ) )
        {
            osg::notify( osg::FATAL ) << "Usage: testconstraint [--debug] [--ctest] <testname>" << std::endl;
            osg::notify( osg::FATAL ) << "\tIf \"--ctest\" is present, <testname> must be present." << std::endl;
            osg::notify( osg::FATAL ) << "\tIf \"--ctest\" is not present, <testname> is optional and defaults to \"Slider\"." << std::endl;
            return( 1 );
        }

        const int returnValue( runCTest( arguments[ ctestPos+1 ] ) );
        if( returnValue == 0 )
            osg::notify( osg::ALWAYS ) << "Test \"" << arguments[ ctestPos+1 ] << "\" passed." << std::endl;
        return( returnValue );
    }

    const bool debugDisplay( arguments.find( "--debug" ) > 0 );


    btDiscreteDynamicsWorld* bulletWorld = initPhysics();
    osg::Group* root = new osg::Group;

    // Add ground
    const osg::Vec4 plane( 0., 0., 1., -1.5 );
    osg::Node* groundRoot = osgbDynamics::generateGroundPlane( plane, bulletWorld );
    groundRoot->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    root->addChild( groundRoot );


    osg::Node* nodeA = osgDB::readNodeFile( "tetra.osg" );
    if( nodeA == NULL )
        return( 1 );

    osgwTools::AbsoluteModelTransform* amtA = new osgwTools::AbsoluteModelTransform;
    amtA->setDataVariance( osg::Object::DYNAMIC );
    amtA->addChild( nodeA );
    root->addChild( amtA );

    osg::ref_ptr< osgbDynamics::CreationRecord > crA = new osgbDynamics::CreationRecord;
    crA->_sceneGraph = amtA;
    crA->_shapeType = CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE;
    crA->_mass = 0.5;


    osg::Node* nodeB = osgDB::readNodeFile( "block.osg" );
    if( nodeB == NULL )
        return( 1 );

    osgwTools::AbsoluteModelTransform* amtB = new osgwTools::AbsoluteModelTransform;
    amtB->setDataVariance( osg::Object::DYNAMIC );
    amtB->addChild( nodeB );
    root->addChild( amtB );

    osg::ref_ptr< osgbDynamics::CreationRecord > crB = new osgbDynamics::CreationRecord;
    crB->_sceneGraph = amtB;
    crB->_shapeType = BOX_SHAPE_PROXYTYPE;
    crB->_mass = 4.;


    osg::Node* nodeC = osgDB::readNodeFile( "dice.osg" );
    if( nodeC == NULL )
        return( 1 );

    osgwTools::AbsoluteModelTransform* amtC = new osgwTools::AbsoluteModelTransform;
    amtC->setDataVariance( osg::Object::DYNAMIC );
    amtC->addChild( nodeC );
    root->addChild( amtC );

    osg::ref_ptr< osgbDynamics::CreationRecord > crC = new osgbDynamics::CreationRecord;
    crC->_sceneGraph = amtC;
    crC->_shapeType = CONVEX_HULL_SHAPE_PROXYTYPE;
    crC->_mass = 1.5;


    osg::Node* nodeD = osgDB::readNodeFile( "axes.osg" );
    if( nodeD == NULL )
        return( 1 );

    osgwTools::AbsoluteModelTransform* amtD = new osgwTools::AbsoluteModelTransform;
    amtD->setDataVariance( osg::Object::DYNAMIC );
    amtD->addChild( nodeD );
    root->addChild( amtD );

    osg::ref_ptr< osgbDynamics::CreationRecord > crD = new osgbDynamics::CreationRecord;
    crD->_sceneGraph = amtD;
    crD->_shapeType = BOX_SHAPE_PROXYTYPE;


    osg::Node* nodeE = osgDB::readNodeFile( "offcube.osg" );
    if( nodeE == NULL )
        return( 1 );

    osgwTools::AbsoluteModelTransform* amtE = new osgwTools::AbsoluteModelTransform;
    amtE->setDataVariance( osg::Object::DYNAMIC );
    amtE->addChild( nodeE );
    root->addChild( amtE );

    osg::ref_ptr< osgbDynamics::CreationRecord > crE = new osgbDynamics::CreationRecord;
    crE->_sceneGraph = amtE;
    crE->_shapeType = BOX_SHAPE_PROXYTYPE;


    if( arguments.find( "LinearSpring" ) > 0 )
    {
        osg::Matrix bXform = osg::Matrix::rotate( osg::PI_4, 0., 0., 1. ) *
            osg::Matrix::translate( -3., 1., 0. );
        crB->_parentTransform = bXform;
        btRigidBody* rbB = osgbDynamics::createRigidBody( crB.get() );
        amtB->setUserData( new osgbCollision::RefRigidBody( rbB ) );
        bulletWorld->addRigidBody( rbB );

        osg::Matrix cXform = osg::Matrix::translate( 5., 0., 0. ) *
            osg::Matrix::rotate( osg::PI_4, 0., 0., 1. ) *
            osg::Matrix::translate( -3., 1., 0. );
        crC->_parentTransform = cXform;
        btRigidBody* rbC = osgbDynamics::createRigidBody( crC.get() );
        amtC->setUserData( new osgbCollision::RefRigidBody( rbC ) );
        bulletWorld->addRigidBody( rbC );

        osg::Matrix aXform = osg::Matrix::rotate( -osg::PI_4, 0., 0., 1. ) *
            osg::Matrix::translate( 4., 3., 0. );
        crA->_parentTransform = aXform;
        crA->_mass = 0.;
        btRigidBody* rbA = osgbDynamics::createRigidBody( crA.get() );
        amtA->setUserData( new osgbCollision::RefRigidBody( rbA ) );
        bulletWorld->addRigidBody( rbA );

        osg::Matrix eXform = osg::Matrix::translate( 0.05, 0., -1.95 ) *
            osg::Matrix::rotate( -osg::PI_4, 0., 0., 1. ) *
            osg::Matrix::translate( 4., 3., 0. );
        crE->_parentTransform = eXform;
        btRigidBody* rbE = osgbDynamics::createRigidBody( crE.get() );
        amtE->setUserData( new osgbCollision::RefRigidBody( rbE ) );
        bulletWorld->addRigidBody( rbE );

        {
            osg::Vec3 axis( .707, .707, 0. );

            osg::ref_ptr< osgbDynamics::LinearSpringConstraint > cons1 = new osgbDynamics::LinearSpringConstraint(
                rbB, bXform, rbC, cXform, axis );
            bulletWorld->addConstraint( cons1->getConstraint() );

            axis.set( -.707, 0., 1. );
            osg::ref_ptr< osgbDynamics::LinearSpringConstraint > cons2 = new osgbDynamics::LinearSpringConstraint(
                rbA, aXform, rbE, eXform, axis );
            cons2->setStiffness( 50. );
            cons2->setLimit( -1., 2. );
            bulletWorld->addConstraint( cons2->getConstraint() );
        }
    }
    else if( arguments.find( "AngleSpring" ) > 0 )
    {
        osg::Matrix bXform = osg::Matrix::rotate( osg::PI_4, 0., 0., 1. ) *
            osg::Matrix::translate( -3., 1., 0. );
        crB->_parentTransform = bXform;
        btRigidBody* rbB = osgbDynamics::createRigidBody( crB.get() );
        amtB->setUserData( new osgbCollision::RefRigidBody( rbB ) );
        bulletWorld->addRigidBody( rbB );

        osg::Matrix cXform = osg::Matrix::translate( 5., 0., 0. ) *
            osg::Matrix::rotate( osg::PI_4, 0., 0., 1. ) *
            osg::Matrix::translate( -3., 1., 0. );
        crC->_parentTransform = cXform;
        btRigidBody* rbC = osgbDynamics::createRigidBody( crC.get() );
        amtC->setUserData( new osgbCollision::RefRigidBody( rbC ) );
        bulletWorld->addRigidBody( rbC );

        osg::Matrix aXform = osg::Matrix::rotate( -osg::PI_4, 0., 0., 1. ) *
            osg::Matrix::translate( 4., 3., 0. );
        crA->_parentTransform = aXform;
        crA->_mass = 0.;
        btRigidBody* rbA = osgbDynamics::createRigidBody( crA.get() );
        amtA->setUserData( new osgbCollision::RefRigidBody( rbA ) );
        bulletWorld->addRigidBody( rbA );

        osg::Matrix eXform = osg::Matrix::translate( 0.05, 0., -1.95 ) *
            osg::Matrix::rotate( -osg::PI_4, 0., 0., 1. ) *
            osg::Matrix::translate( 4., 3., 0. );
        crE->_parentTransform = eXform;
        btRigidBody* rbE = osgbDynamics::createRigidBody( crE.get() );
        amtE->setUserData( new osgbCollision::RefRigidBody( rbE ) );
        bulletWorld->addRigidBody( rbE );

        {
            osg::Vec3 axis( .707, .707, 0. );
            osg::Vec3 point = osg::Vec3( 0., 0., 0.05 ) * bXform;

            osg::ref_ptr< osgbDynamics::AngleSpringConstraint > cons1 = new osgbDynamics::AngleSpringConstraint(
                rbB, bXform, rbC, cXform, axis, point );
            bulletWorld->addConstraint( cons1->getConstraint() );

            axis.set( -.707, 0., 1. );
            point.set( osg::Vec3( 1.05, 1.05, -1. ) * aXform );
            osg::ref_ptr< osgbDynamics::AngleSpringConstraint > cons2 = new osgbDynamics::AngleSpringConstraint(
                rbA, aXform, rbE, eXform, axis, point );
            cons2->setStiffness( 20. );
            cons2->setLimit( -.7, .7 );
            bulletWorld->addConstraint( cons2->getConstraint() );
        }
    }
    else if( arguments.find( "LinearAngleSpring" ) > 0 )
    {
        osg::Matrix bXform = osg::Matrix::rotate( osg::PI_4, 0., 0., 1. ) *
            osg::Matrix::translate( -3., 1., 0. );
        crB->_parentTransform = bXform;
        btRigidBody* rbB = osgbDynamics::createRigidBody( crB.get() );
        amtB->setUserData( new osgbCollision::RefRigidBody( rbB ) );
        bulletWorld->addRigidBody( rbB );

        osg::Matrix cXform = osg::Matrix::translate( 5., 0., 0. ) *
            osg::Matrix::rotate( osg::PI_4, 0., 0., 1. ) *
            osg::Matrix::translate( -3., 1., 0. );
        crC->_parentTransform = cXform;
        btRigidBody* rbC = osgbDynamics::createRigidBody( crC.get() );
        amtC->setUserData( new osgbCollision::RefRigidBody( rbC ) );
        bulletWorld->addRigidBody( rbC );

        osg::Matrix aXform = osg::Matrix::rotate( -osg::PI_4, 0., 0., 1. ) *
            osg::Matrix::translate( 4., 3., 0. );
        crA->_parentTransform = aXform;
        crA->_mass = 0.;
        btRigidBody* rbA = osgbDynamics::createRigidBody( crA.get() );
        amtA->setUserData( new osgbCollision::RefRigidBody( rbA ) );
        bulletWorld->addRigidBody( rbA );

        osg::Matrix eXform = osg::Matrix::translate( 0.05, 0., -1.95 ) *
            osg::Matrix::rotate( -osg::PI_4, 0., 0., 1. ) *
            osg::Matrix::translate( 4., 3., 0. );
        crE->_parentTransform = eXform;
        btRigidBody* rbE = osgbDynamics::createRigidBody( crE.get() );
        amtE->setUserData( new osgbCollision::RefRigidBody( rbE ) );
        bulletWorld->addRigidBody( rbE );

        {
            osg::Vec3 axis( .707, .707, 0. );
            osg::Vec3 point = osg::Vec3( 0., 0., 0.05 ) * bXform;

            osg::ref_ptr< osgbDynamics::LinearAngleSpringConstraint > cons1 = new osgbDynamics::LinearAngleSpringConstraint(
                rbB, bXform, rbC, cXform, axis, point );
            cons1->setLinearLimit( -1.5, 3. );
            bulletWorld->addConstraint( cons1->getConstraint() );

            axis.set( -.707, 0., 1. );
            point.set( osg::Vec3( 1.05, 1.05, -1. ) * aXform );
            osg::ref_ptr< osgbDynamics::LinearAngleSpringConstraint > cons2 = new osgbDynamics::LinearAngleSpringConstraint(
                rbA, aXform, rbE, eXform, axis, point );
            cons2->setLinearStiffness( 30. );
            cons2->setAngleStiffness( 60. );
            cons2->setAngleLimit( -.7, .7 );
            bulletWorld->addConstraint( cons2->getConstraint() );
        }
    }
    else if( arguments.find( "Cardan" ) > 0 )
    {
        osg::Matrix cXform = osg::Matrix::rotate( .5, 0., 1., 0. ) *
            osg::Matrix::translate( 0., 0., 8. );
        crC->_parentTransform = cXform;
        btRigidBody* rbC = osgbDynamics::createRigidBody( crC.get() );
        amtC->setUserData( new osgbCollision::RefRigidBody( rbC ) );
        bulletWorld->addRigidBody( rbC );

        osg::Vec3 anchor1 = osg::Vec3( 0., 0., -1.5 ) * cXform;

        osg::Matrix eXform = osg::Matrix::translate( anchor1[0]-2., -2., 3. );
        crE->_parentTransform = eXform;
        btRigidBody* rbE = osgbDynamics::createRigidBody( crE.get() );
        amtE->setUserData( new osgbCollision::RefRigidBody( rbE ) );
        bulletWorld->addRigidBody( rbE );

        {
            // Use Cardan to constraint C and E together.
            osg::Vec3 axisA( 0., 1., 0. );
            osg::Vec3 axisB( -1., 0., 0. );

            osg::ref_ptr< osgbDynamics::CardanConstraint > cons1 = new osgbDynamics::CardanConstraint(
                rbC, cXform, rbE, eXform, axisA, axisB, anchor1 );
            bulletWorld->addConstraint( cons1->getConstraint() );

            // Further constraint C in space so that E hangs off it,
            // and allows user to rotate C. Torque should transfer to E.
            osg::Vec3 eAxis = osg::Vec3( 0., 0., 1. ) * osg::Matrix::rotate( .5, 0., 1., 0. );
            osg::Vec2 linLimits( 0., 0. );
            osg::Vec2 angLimits( -osg::PI, osg::PI );
            osg::ref_ptr< osgbDynamics::TwistSliderConstraint > cons2 = new osgbDynamics::TwistSliderConstraint(
                rbC, cXform, eAxis, anchor1, linLimits, angLimits );
            bulletWorld->addConstraint( cons2->getConstraint() );
        }
    }
    else if( arguments.find( "Ragdoll" ) > 0 )
    {
        osg::Matrix cXform = osg::Matrix::translate( 3., 4., 0. );
        crC->_parentTransform = cXform;
        btRigidBody* rbC = osgbDynamics::createRigidBody( crC.get() );
        amtC->setUserData( new osgbCollision::RefRigidBody( rbC ) );
        bulletWorld->addRigidBody( rbC );

        osg::Matrix eXform = osg::Matrix::translate( 4., 2., -2. );
        crE->_parentTransform = eXform;
        btRigidBody* rbE = osgbDynamics::createRigidBody( crE.get() );
        amtE->setUserData( new osgbCollision::RefRigidBody( rbE ) );
        bulletWorld->addRigidBody( rbE );

        osg::Matrix bXform = osg::Matrix::translate( -3., 1., 1. );
        crB->_parentTransform = bXform;
        btRigidBody* rbB = osgbDynamics::createRigidBody( crB.get() );
        amtB->setUserData( new osgbCollision::RefRigidBody( rbB ) );
        bulletWorld->addRigidBody( rbB );

        osg::Matrix aXform = osg::Matrix::translate( 0., 0., 5. );
        crA->_parentTransform = aXform;
        btRigidBody* rbA = osgbDynamics::createRigidBody( crA.get() );
        amtA->setUserData( new osgbCollision::RefRigidBody( rbA ) );
        bulletWorld->addRigidBody( rbA );

        {
            osg::Vec3 point( 4.5, 4., 0. );
            osg::Vec3 axis( -1., 0., 0. );
            double angle( .75 );

            osg::ref_ptr< osgbDynamics::RagdollConstraint > cons1 = new osgbDynamics::RagdollConstraint(
                rbC, cXform, rbE, eXform, point, axis, angle );
            bulletWorld->addConstraint( cons1->getConstraint() );


            // TBD There seems to be an issue with specifying only one rigid body
            // when using btConeTwistConstraint. Could be an osgBullet issue, or
            // could be a Bullet issue. Need to investigate.

            point.set( -3., 1., 2.5 );
            axis.set( 0., 0., -1. );
            angle = osg::PI_2;

            osg::ref_ptr< osgbDynamics::RagdollConstraint > cons2 = new osgbDynamics::RagdollConstraint(
                rbB, bXform, point, axis, angle );
            bulletWorld->addConstraint( cons2->getConstraint() );

            point.set( -1., 0., 5. );
            axis.set( 1., 0., 0. );
            angle = osg::PI_2;

            osg::ref_ptr< osgbDynamics::RagdollConstraint > cons3 = new osgbDynamics::RagdollConstraint(
                rbA, aXform, point, axis, angle );
            bulletWorld->addConstraint( cons3->getConstraint() );
        }
    }
    else if( arguments.find( "WheelSuspension" ) > 0 )
    {
        osg::Matrix aXform = osg::Matrix::translate( 0., 10., 0. );
        crA->_parentTransform = aXform;
        btRigidBody* rbA = osgbDynamics::createRigidBody( crA.get() );
        amtA->setUserData( new osgbCollision::RefRigidBody( rbA ) );
        bulletWorld->addRigidBody( rbA );

        osg::Matrix eXform = osg::Matrix::translate( 4., 10., 0. );
        crE->_parentTransform = eXform;
        btRigidBody* rbE = osgbDynamics::createRigidBody( crE.get() );
        amtE->setUserData( new osgbCollision::RefRigidBody( rbE ) );
        bulletWorld->addRigidBody( rbE );

        osg::Matrix cXform = osg::Matrix::translate( 0., -3., 1. );
        crC->_parentTransform = cXform;
        btRigidBody* rbC = osgbDynamics::createRigidBody( crC.get() );
        amtC->setUserData( new osgbCollision::RefRigidBody( rbC ) );
        bulletWorld->addRigidBody( rbC );

        osg::Matrix bXform = osg::Matrix::rotate( osg::PI_2, 0., 1., 0. ) *
            osg::Matrix::translate( 0., 0., 1.5 );
        crB->_parentTransform = bXform;
        crB->_mass = 0.;
        btRigidBody* rbB = osgbDynamics::createRigidBody( crB.get() );
        amtB->setUserData( new osgbCollision::RefRigidBody( rbB ) );
        bulletWorld->addRigidBody( rbB );

        {
            osg::Vec3 springAxis( 0., 0., 1. );
            osg::Vec3 axleAxis( 0., 1., 0. );
            osg::Vec2 linLimit( -3., 3. );
            osg::Vec2 angleLimit( -.2, .2 );
            osg::Vec3 anchor( 0., -3., 1. );

            osg::ref_ptr< osgbDynamics::WheelSuspensionConstraint > cons1 = new osgbDynamics::WheelSuspensionConstraint(
                rbB, rbC, springAxis, axleAxis, linLimit, angleLimit, anchor );
            bulletWorld->addConstraint( cons1->getConstraint() );
        }
    }
    else if( arguments.find( "Hinge" ) > 0 )
    {
        osg::Matrix aXform = osg::Matrix::translate( 0., 10., 0. );
        crA->_parentTransform = aXform;
        btRigidBody* rbA = osgbDynamics::createRigidBody( crA.get() );
        amtA->setUserData( new osgbCollision::RefRigidBody( rbA ) );
        bulletWorld->addRigidBody( rbA );

        osg::Matrix cXform = osg::Matrix::rotate( osg::PI_4, 0., 0., 1. ) *
            osg::Matrix::translate( 0., 8., 2. );
        crC->_parentTransform = cXform;
        btRigidBody* rbC = osgbDynamics::createRigidBody( crC.get() );
        amtC->setUserData( new osgbCollision::RefRigidBody( rbC ) );
        bulletWorld->addRigidBody( rbC );

        osg::Matrix bXform = osg::Matrix::identity();
        crB->_parentTransform = bXform;
        btRigidBody* rbB = osgbDynamics::createRigidBody( crB.get() );
        amtB->setUserData( new osgbCollision::RefRigidBody( rbB ) );
        bulletWorld->addRigidBody( rbB );

        osg::Matrix eXform = osg::Matrix::translate( 2., -2., .5 );
        crE->_parentTransform = eXform;
        btRigidBody* rbE = osgbDynamics::createRigidBody( crE.get() );
        amtE->setUserData( new osgbCollision::RefRigidBody( rbE ) );
        bulletWorld->addRigidBody( rbE );

        {
            osg::Vec3 point( 1., 8., 0. );
            osg::Vec3 axis( 0., 0., 1. );
            osg::Vec2 limit( -osg::PI_2, osg::PI_2 );

            osg::ref_ptr< osgbDynamics::HingeConstraint > cons1 = new osgbDynamics::HingeConstraint(
                rbC, cXform, axis, point, limit );
            bulletWorld->addConstraint( cons1->getConstraint() );

            axis.set( 0., 1., 0. );
            point.set( 3., 0., 1.5 );
            limit.set( -osg::PI_2, osg::PI );
            osg::ref_ptr< osgbDynamics::HingeConstraint > cons2 = new osgbDynamics::HingeConstraint(
                rbE, eXform, rbB, bXform, axis, point, limit );
            bulletWorld->addConstraint( cons2->getConstraint() );
        }
    }
    else if( arguments.find( "Planar" ) > 0 )
    {
        osg::Matrix aXform = osg::Matrix::translate( 0., 10., 3. );
        crA->_parentTransform = aXform;
        btRigidBody* rbA = osgbDynamics::createRigidBody( crA.get() );
        amtA->setUserData( new osgbCollision::RefRigidBody( rbA ) );
        bulletWorld->addRigidBody( rbA );

        osg::Matrix cXform = osg::Matrix::rotate( osg::PI_4, 0., 0., 1. ) *
            osg::Matrix::translate( 0., 8., 2. );
        crC->_parentTransform = cXform;
        btRigidBody* rbC = osgbDynamics::createRigidBody( crC.get() );
        amtC->setUserData( new osgbCollision::RefRigidBody( rbC ) );
        bulletWorld->addRigidBody( rbC );

        osg::Matrix bXform = osg::Matrix::identity();
        crB->_parentTransform = bXform;
        btRigidBody* rbB = osgbDynamics::createRigidBody( crB.get() );
        amtB->setUserData( new osgbCollision::RefRigidBody( rbB ) );
        bulletWorld->addRigidBody( rbB );

        osg::Matrix eXform = osg::Matrix::translate( -2., -2., 1. ) *
            osg::Matrix::rotate( osg::PI_4, 0., 0., 1. );
        crE->_parentTransform = eXform;
        btRigidBody* rbE = osgbDynamics::createRigidBody( crE.get() );
        amtE->setUserData( new osgbCollision::RefRigidBody( rbE ) );
        bulletWorld->addRigidBody( rbE );

        {
            osg::Vec2 loLimit( -4., -1. );
            osg::Vec2 hiLimit( 2., 1. );
            osg::Matrix orient = osg::Matrix::rotate( osg::PI_4, 0., 0., 1. );

            osg::ref_ptr< osgbDynamics::PlanarConstraint > cons1 = new osgbDynamics::PlanarConstraint(
                rbC, cXform, loLimit, hiLimit, orient );
            bulletWorld->addConstraint( cons1->getConstraint() );

            orient = osg::Matrix::rotate( .1, 0., 1., 0. );
            osg::ref_ptr< osgbDynamics::PlanarConstraint > cons2 = new osgbDynamics::PlanarConstraint(
                rbB, bXform, rbE, eXform, loLimit, hiLimit, orient );
            bulletWorld->addConstraint( cons2->getConstraint() );
        }
    }
    else if( arguments.find( "Box" ) > 0 )
    {
        osg::Matrix aXform = osg::Matrix::translate( 0., 10., 3. );
        crA->_parentTransform = aXform;
        btRigidBody* rbA = osgbDynamics::createRigidBody( crA.get() );
        amtA->setUserData( new osgbCollision::RefRigidBody( rbA ) );
        bulletWorld->addRigidBody( rbA );

        osg::Matrix cXform = osg::Matrix::rotate( osg::PI_4, 0., 0., 1. ) *
            osg::Matrix::translate( 0., 8., 2. );
        crC->_parentTransform = cXform;
        btRigidBody* rbC = osgbDynamics::createRigidBody( crC.get() );
        amtC->setUserData( new osgbCollision::RefRigidBody( rbC ) );
        bulletWorld->addRigidBody( rbC );

        osg::Matrix bXform = osg::Matrix::identity();
        crB->_parentTransform = bXform;
        btRigidBody* rbB = osgbDynamics::createRigidBody( crB.get() );
        amtB->setUserData( new osgbCollision::RefRigidBody( rbB ) );
        bulletWorld->addRigidBody( rbB );

        osg::Matrix eXform = osg::Matrix::translate( -2., -2., 1. ) *
            osg::Matrix::rotate( osg::PI_4, 0., 0., 1. );
        crE->_parentTransform = eXform;
        btRigidBody* rbE = osgbDynamics::createRigidBody( crE.get() );
        amtE->setUserData( new osgbCollision::RefRigidBody( rbE ) );
        bulletWorld->addRigidBody( rbE );

        {
            osg::Vec3 loLimit( -4., -1., -1. );
            osg::Vec3 hiLimit( 2., 1., 5. );
            osg::Matrix orient = osg::Matrix::rotate( osg::PI_4, 0., 0., 1. );

            osg::ref_ptr< osgbDynamics::BoxConstraint > cons1 = new osgbDynamics::BoxConstraint(
                rbC, cXform, loLimit, hiLimit, orient );
            bulletWorld->addConstraint( cons1->getConstraint() );

            orient = osg::Matrix::rotate( .1, 0., 1., 0. );
            osg::ref_ptr< osgbDynamics::BoxConstraint > cons2 = new osgbDynamics::BoxConstraint(
                rbE, eXform, rbB, bXform, loLimit, hiLimit, orient );
            bulletWorld->addConstraint( cons2->getConstraint() );
        }
    }
    else if( arguments.find( "Fixed" ) > 0 )
    {
        osg::Matrix aXform = osg::Matrix::translate( 0., 0., 3. );
        crA->_parentTransform = aXform;
        btRigidBody* rbA = osgbDynamics::createRigidBody( crA.get() );
        amtA->setUserData( new osgbCollision::RefRigidBody( rbA ) );
        bulletWorld->addRigidBody( rbA );

        osg::Matrix bXform = osg::Matrix::identity();
        crB->_parentTransform = bXform;
        btRigidBody* rbB = osgbDynamics::createRigidBody( crB.get() );
        amtB->setUserData( new osgbCollision::RefRigidBody( rbB ) );
        bulletWorld->addRigidBody( rbB );

        osg::Matrix cXform = osg::Matrix::rotate( osg::PI_4, osg::Vec3( 0., 0., 1. ) ) *
            osg::Matrix::translate( 2., 12., 5. );
        crC->_parentTransform = cXform;
        btRigidBody* rbC = osgbDynamics::createRigidBody( crC.get() );
        amtC->setUserData( new osgbCollision::RefRigidBody( rbC ) );
        bulletWorld->addRigidBody( rbC );

        osg::Matrix eXform = osg::Matrix::translate( -5., -2., 1. );
        crE->_parentTransform = eXform;
        btRigidBody* rbE = osgbDynamics::createRigidBody( crE.get() );
        amtE->setUserData( new osgbCollision::RefRigidBody( rbE ) );
        bulletWorld->addRigidBody( rbE );

        {
            osg::ref_ptr< osgbDynamics::FixedConstraint > cons0 = new osgbDynamics::FixedConstraint(
                rbB, bXform, rbA, aXform );
            bulletWorld->addConstraint( cons0->getConstraint() );

            osg::ref_ptr< osgbDynamics::FixedConstraint > cons1 = new osgbDynamics::FixedConstraint(
                rbC, cXform );
            bulletWorld->addConstraint( cons1->getConstraint() );

            osg::ref_ptr< osgbDynamics::FixedConstraint > cons2 = new osgbDynamics::FixedConstraint(
                rbE, eXform, rbB, bXform );
            bulletWorld->addConstraint( cons2->getConstraint() );
        }
    }
    else if( arguments.find( "BallAndSocket" ) > 0 )
    {
        osg::Matrix aXform = osg::Matrix::translate( 1., 1., 2.5 );
        crA->_parentTransform = aXform;
        btRigidBody* rbA = osgbDynamics::createRigidBody( crA.get() );
        amtA->setUserData( new osgbCollision::RefRigidBody( rbA ) );
        bulletWorld->addRigidBody( rbA );

        osg::Matrix bXform = osg::Matrix::identity();
        crB->_parentTransform = bXform;
        btRigidBody* rbB = osgbDynamics::createRigidBody( crB.get() );
        amtB->setUserData( new osgbCollision::RefRigidBody( rbB ) );
        bulletWorld->addRigidBody( rbB );

        osg::Matrix cXform = osg::Matrix::translate( 2., 12., 5. );
        crC->_parentTransform = cXform;
        btRigidBody* rbC = osgbDynamics::createRigidBody( crC.get() );
        amtC->setUserData( new osgbCollision::RefRigidBody( rbC ) );
        bulletWorld->addRigidBody( rbC );

        osg::Matrix dXform = osg::Matrix::rotate( .2, osg::Vec3( 0., 0., 1. ) ) *
            osg::Matrix::translate( -9., 0., 0. );
        crD->_parentTransform = dXform;
        btRigidBody* rbD = osgbDynamics::createRigidBody( crD.get() );
        amtD->setUserData( new osgbCollision::RefRigidBody( rbD ) );
        bulletWorld->addRigidBody( rbD );

        osg::Matrix eXform = osg::Matrix::translate( 3., 10., 3. );
        crE->_parentTransform = eXform;
        btRigidBody* rbE = osgbDynamics::createRigidBody( crE.get() );
        amtE->setUserData( new osgbCollision::RefRigidBody( rbE ) );
        bulletWorld->addRigidBody( rbE );

        {
            osg::Vec3 point( 0., 0., 1.5 );
            osg::ref_ptr< osgbDynamics::BallAndSocketConstraint > cons0 = new osgbDynamics::BallAndSocketConstraint(
                rbB, bXform, rbA, aXform, point );
            bulletWorld->addConstraint( cons0->getConstraint() );

            point.set( 3.5, 12., 5. );
            osg::ref_ptr< osgbDynamics::BallAndSocketConstraint > cons1 = new osgbDynamics::BallAndSocketConstraint(
                rbC, cXform, rbE, eXform, point );
            bulletWorld->addConstraint( cons1->getConstraint() );

            // Make point correspond roughly to (1., 0., 1.) in local coords)
            point = osg::Vec3( 1., 0., 1. ) * dXform;
            osg::ref_ptr< osgbDynamics::BallAndSocketConstraint > cons2 = new osgbDynamics::BallAndSocketConstraint(
                rbD, dXform, point );
            bulletWorld->addConstraint( cons2->getConstraint() );
        }
    }
    else if( arguments.find( "TwistSlider" ) > 0 )
    {
        osg::Matrix aXform = osg::Matrix::translate( -4., 8., 3. );
        crA->_parentTransform = aXform;
        btRigidBody* rbA = osgbDynamics::createRigidBody( crA.get() );
        amtA->setUserData( new osgbCollision::RefRigidBody( rbA ) );
        bulletWorld->addRigidBody( rbA );

        osg::Matrix bXform = osg::Matrix::translate( 1., 0., 0. );
        crB->_parentTransform = bXform;
        btRigidBody* rbB = osgbDynamics::createRigidBody( crB.get() );
        amtB->setUserData( new osgbCollision::RefRigidBody( rbB ) );
        bulletWorld->addRigidBody( rbB );

        osg::Matrix cXform = osg::Matrix::rotate( osg::PI_4, osg::Vec3( 0., 0., 1. ) ) *
            osg::Matrix::translate( 2., 12., 5. );
        crC->_parentTransform = cXform;
        btRigidBody* rbC = osgbDynamics::createRigidBody( crC.get() );
        amtC->setUserData( new osgbCollision::RefRigidBody( rbC ) );
        bulletWorld->addRigidBody( rbC );

        osg::Matrix eXform = osg::Matrix::translate( -1., -2., 1.5 );
        crE->_parentTransform = eXform;
        btRigidBody* rbE = osgbDynamics::createRigidBody( crE.get() );
        amtE->setUserData( new osgbCollision::RefRigidBody( rbE ) );
        bulletWorld->addRigidBody( rbE );

        {
            osg::Vec3 axis( 0., 1., .1 );
            osg::Vec3 point( 1., 0., 3.5 );
            osg::Vec2 linLimits( -1., 1. );
            osg::Vec2 angLimits( -1., 1. );
            osg::ref_ptr< osgbDynamics::TwistSliderConstraint > cons0 = new osgbDynamics::TwistSliderConstraint(
                rbE, eXform, rbB, bXform, axis, point, linLimits, angLimits );
            bulletWorld->addConstraint( cons0->getConstraint() );

            axis.set( 0., 1., 0. );
            point.set( 2., 12., 5.1 );
            linLimits.set( -3., 3. );
            angLimits.set( -osg::PI, osg::PI );
            osg::ref_ptr< osgbDynamics::TwistSliderConstraint > cons1 = new osgbDynamics::TwistSliderConstraint(
                rbC, cXform, axis, point, linLimits, angLimits );
            bulletWorld->addConstraint( cons1->getConstraint() );
        }
    }
    else // "Slider" by default.
    {
        osg::Matrix aXform = osg::Matrix::translate( 0., 0., 3. );
        crA->_parentTransform = aXform;
        btRigidBody* rbA = osgbDynamics::createRigidBody( crA.get() );
        amtA->setUserData( new osgbCollision::RefRigidBody( rbA ) );
        bulletWorld->addRigidBody( rbA );

        osg::Matrix bXform = osg::Matrix::identity();
        crB->_parentTransform = bXform;
        btRigidBody* rbB = osgbDynamics::createRigidBody( crB.get() );
        amtB->setUserData( new osgbCollision::RefRigidBody( rbB ) );
        bulletWorld->addRigidBody( rbB );

        osg::Matrix cXform = osg::Matrix::rotate( osg::PI_4, osg::Vec3( 0., 0., 1. ) ) *
            osg::Matrix::translate( 2., 12., 5. );
        crC->_parentTransform = cXform;
        btRigidBody* rbC = osgbDynamics::createRigidBody( crC.get() );
        amtC->setUserData( new osgbCollision::RefRigidBody( rbC ) );
        bulletWorld->addRigidBody( rbC );

        osg::Matrix eXform = osg::Matrix::translate( -9., -2., -2. );
        crE->_parentTransform = eXform;
        btRigidBody* rbE = osgbDynamics::createRigidBody( crE.get() );
        amtE->setUserData( new osgbCollision::RefRigidBody( rbE ) );
        bulletWorld->addRigidBody( rbE );

        {
            osg::Vec3 axis( 0., 1., .1 );
            osg::Vec2 limits( -1., 1. );
            osg::ref_ptr< osgbDynamics::SliderConstraint > cons0 = new osgbDynamics::SliderConstraint(
                rbA, aXform, rbB, bXform, axis, limits );
            bulletWorld->addConstraint( cons0->getConstraint() );

            axis.set( -1., -1., 0. );
            limits.set( -3., 3. );
            osg::ref_ptr< osgbDynamics::SliderConstraint > cons1 = new osgbDynamics::SliderConstraint(
                rbC, cXform, axis, limits );
            bulletWorld->addConstraint( cons1->getConstraint() );

            axis.set( 0., 1., 0. );
            limits.set( -1., 1. );
            osg::ref_ptr< osgbDynamics::SliderConstraint > cons2 = new osgbDynamics::SliderConstraint(
                rbE, eXform, axis, limits );
            bulletWorld->addConstraint( cons2->getConstraint() );
        }
    }


    osgbCollision::GLDebugDrawer* dbgDraw( NULL );
    if( debugDisplay )
    {
        dbgDraw = new osgbCollision::GLDebugDrawer();
        dbgDraw->setDebugMode( ~btIDebugDraw::DBG_DrawText );
        bulletWorld->setDebugDrawer( dbgDraw );
        osg::Node* dbgRoot = dbgDraw->getSceneGraph();
        root->addChild( dbgRoot );
    }


    osgViewer::Viewer viewer( arguments );
    viewer.setUpViewInWindow( 30, 30, 800, 450 );
    viewer.setSceneData( root );

    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    tb->setHomePosition( osg::Vec3( 0., -26., 4. ), osg::Vec3( 0., 0., 1.5 ), osg::Vec3( 0., 0., 1. ) ); 
    viewer.setCameraManipulator( tb );
    viewer.getCamera()->setClearColor( osg::Vec4( .5, .5, .5, 1. ) );

    viewer.addEventHandler( new osgbInteraction::DragHandler(
        bulletWorld, viewer.getCamera() ) );

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
