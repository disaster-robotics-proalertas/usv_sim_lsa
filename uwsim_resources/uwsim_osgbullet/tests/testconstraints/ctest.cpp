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
#include <osgbDynamics/Constraints.h>
#include <osgbDynamics/RigidBody.h>
#include <osgwTools/AbsoluteModelTransform.h>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Notify>

#include <btBulletDynamicsCommon.h>


// Shorthand to display a message and return an error.
#define ERROR( name, msg ) \
    { \
        osg::notify( osg::WARN ) << name << " " << msg << std::endl; \
        return( 1 ); \
    }


int runCTest( const std::string& testName )
{
    const std::string fileName( "testconstraint.osg" );

    // Create two rigid bodies for testing.

    osg::ref_ptr< osg::Group > root = new osg::Group;

    osg::Node* node = osgDB::readNodeFile( "tetra.osg" );
    if( node == NULL )
        ERROR("Init:","Can't load model data file.");
    osg::Matrix aXform = osg::Matrix::translate( 4., 2., 0. );

    osgwTools::AbsoluteModelTransform* amt = new osgwTools::AbsoluteModelTransform;
    amt->setDataVariance( osg::Object::DYNAMIC );
    amt->addChild( node );
    root->addChild( amt );

    osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = amt;
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
        cr->_mass = .5;
    cr->_parentTransform = aXform;
    btRigidBody* rbA = osgbDynamics::createRigidBody( cr.get() );


    node = osgDB::readNodeFile( "block.osg" );
    if( node == NULL )
        ERROR("Init:","Can't load model data file.");
    osg::Matrix bXform = osg::Matrix::identity();

    amt = new osgwTools::AbsoluteModelTransform;
    amt->setDataVariance( osg::Object::DYNAMIC );
    amt->addChild( node );
    root->addChild( amt );

    cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = amt;
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
        cr->_mass = 4.;
    cr->_parentTransform = bXform;
    btRigidBody* rbB = osgbDynamics::createRigidBody( cr.get() );

    //
    // SliderConstraint
    if( testName == std::string( "Slider" ) )
    {
        osg::Vec3 axis( 0., 0., 1. );
        osg::Vec2 limits( -4., 4. );
        osg::ref_ptr< osgbDynamics::SliderConstraint > cons = new osgbDynamics::SliderConstraint(
            rbA, aXform, rbB, bXform, axis, limits );

        if( cons->getAsBtSlider() == NULL )
            ERROR(testName,"won't typecast as btSliderConstraint.");

        if( !( osgDB::writeObjectFile( *cons, fileName ) ) )
            ERROR(testName,"writeObjectFile failed.");

        osg::Object* obj = osgDB::readObjectFile( fileName );
        if( obj == NULL )
            ERROR(testName,"readObjectFile returned NULL.");

        osg::ref_ptr< osgbDynamics::SliderConstraint > cons2 = dynamic_cast<
            osgbDynamics::SliderConstraint* >( obj );
        if( !( cons2.valid() ) )
            ERROR(testName,"dynamic_cast after readObjectFile failed.");

        if( *cons2 != *cons )
            // Note matches can fail due to double precision roundoff.
            // For testing, use only 1s and 0s in matrices.
            ERROR(testName,"failed to match.");

        return( 0 );
    }

    //
    // TwistSliderConstraint
    if( testName == std::string( "TwistSlider" ) )
    {
        osg::Vec3 axis( 0., 0., 1. );
        osg::Vec3 point( 1., 2., 3. );
        osg::Vec2 linLimits( -4., 4. );
        osg::Vec2 angLimits( -1., 2. );
        osg::ref_ptr< osgbDynamics::TwistSliderConstraint > cons = new osgbDynamics::TwistSliderConstraint(
            rbA, aXform, rbB, bXform, axis, point, linLimits, angLimits );

        if( cons->getAsBtSlider() == NULL )
            ERROR(testName,"won't typecast as btSliderConstraint.");

        if( !( osgDB::writeObjectFile( *cons, fileName ) ) )
            ERROR(testName,"writeObjectFile failed.");

        osg::Object* obj = osgDB::readObjectFile( fileName );
        if( obj == NULL )
            ERROR(testName,"readObjectFile returned NULL.");

        osg::ref_ptr< osgbDynamics::TwistSliderConstraint > cons2 = dynamic_cast<
            osgbDynamics::TwistSliderConstraint* >( obj );
        if( !( cons2.valid() ) )
            ERROR(testName,"dynamic_cast after readObjectFile failed.");

        if( *cons2 != *cons )
            // Note matches can fail due to double precision roundoff.
            // For testing, use only 1s and 0s in matrices.
            ERROR(testName,"failed to match.");

        return( 0 );
    }

    //
    // BallAndSocketConstraint
    if( testName == std::string( "BallAndSocket" ) )
    {
        osg::Vec3 point( -5., 5., 3. );
        osg::ref_ptr< osgbDynamics::BallAndSocketConstraint > cons = new osgbDynamics::BallAndSocketConstraint(
            rbA, aXform, rbB, bXform, point );

        if( cons->getAsBtPoint2Point() == NULL )
            ERROR(testName,"won't typecast as btPoint2PointConstraint.");

        if( !( osgDB::writeObjectFile( *cons, fileName ) ) )
            ERROR(testName,"writeObjectFile failed.");

        osg::Object* obj = osgDB::readObjectFile( fileName );
        if( obj == NULL )
            ERROR(testName,"readObjectFile returned NULL.");

        osg::ref_ptr< osgbDynamics::BallAndSocketConstraint > cons2 = dynamic_cast<
            osgbDynamics::BallAndSocketConstraint* >( obj );
        if( !( cons2.valid() ) )
            ERROR(testName,"dynamic_cast after readObjectFile failed.");

        if( *cons2 != *cons )
            // Note matches can fail due to double precision roundoff.
            // For testing, use only 1s and 0s in matrices.
            ERROR(testName,"failed to match.");

        return( 0 );
    }

    //
    // FixedConstraint
    if( testName == std::string( "Fixed" ) )
    {
        osg::ref_ptr< osgbDynamics::FixedConstraint > cons = new osgbDynamics::FixedConstraint(
            rbA, aXform, rbB, bXform );

        if( cons->getAsBtGeneric6Dof() == NULL )
            ERROR(testName,"won't typecast as btGeneric6DofConstraint.");

        if( !( osgDB::writeObjectFile( *cons, fileName ) ) )
            ERROR(testName,"writeObjectFile failed.");

        osg::Object* obj = osgDB::readObjectFile( fileName );
        if( obj == NULL )
            ERROR(testName,"readObjectFile returned NULL.");

        osg::ref_ptr< osgbDynamics::FixedConstraint > cons2 = dynamic_cast<
            osgbDynamics::FixedConstraint* >( obj );
        if( !( cons2.valid() ) )
            ERROR(testName,"dynamic_cast after readObjectFile failed.");

        if( *cons2 != *cons )
            // Note matches can fail due to double precision roundoff.
            // For testing, use only 1s and 0s in matrices.
            ERROR(testName,"failed to match.");

        return( 0 );
    }

    //
    // PlanarConstraint
    if( testName == std::string( "Planar" ) )
    {
        osg::Vec2 loLimit( -2., -3. );
        osg::Vec2 hiLimit( 1., 4. );
        osg::Matrix orient( osg::Matrix::identity() );
        osg::ref_ptr< osgbDynamics::PlanarConstraint > cons = new osgbDynamics::PlanarConstraint(
            rbA, aXform, rbB, bXform, loLimit, hiLimit, orient );

        if( cons->getAsBtGeneric6Dof() == NULL )
            ERROR(testName,"won't typecast as btGeneric6DofConstraint.");

        if( !( osgDB::writeObjectFile( *cons, fileName ) ) )
            ERROR(testName,"writeObjectFile failed.");

        osg::Object* obj = osgDB::readObjectFile( fileName );
        if( obj == NULL )
            ERROR(testName,"readObjectFile returned NULL.");

        osg::ref_ptr< osgbDynamics::PlanarConstraint > cons2 = dynamic_cast<
            osgbDynamics::PlanarConstraint* >( obj );
        if( !( cons2.valid() ) )
            ERROR(testName,"dynamic_cast after readObjectFile failed.");

        if( *cons2 != *cons )
            // Note matches can fail due to double precision roundoff.
            // For testing, use only 1s and 0s in matrices.
            ERROR(testName,"failed to match.");

        return( 0 );
    }

    //
    // BoxConstraint
    if( testName == std::string( "Box" ) )
    {
        osg::Vec3 loLimit( -2., -3., 0. );
        osg::Vec3 hiLimit( 1., 4., 2. );
        osg::Matrix orient( osg::Matrix::identity() );
        osg::ref_ptr< osgbDynamics::BoxConstraint > cons = new osgbDynamics::BoxConstraint(
            rbA, aXform, rbB, bXform, loLimit, hiLimit, orient );

        if( cons->getAsBtGeneric6Dof() == NULL )
            ERROR(testName,"won't typecast as btGeneric6DofConstraint.");

        if( !( osgDB::writeObjectFile( *cons, fileName ) ) )
            ERROR(testName,"writeObjectFile failed.");

        osg::Object* obj = osgDB::readObjectFile( fileName );
        if( obj == NULL )
            ERROR(testName,"readObjectFile returned NULL.");

        osg::ref_ptr< osgbDynamics::BoxConstraint > cons2 = dynamic_cast<
            osgbDynamics::BoxConstraint* >( obj );
        if( !( cons2.valid() ) )
            ERROR(testName,"dynamic_cast after readObjectFile failed.");

        if( *cons2 != *cons )
            // Note matches can fail due to double precision roundoff.
            // For testing, use only 1s and 0s in matrices.
            ERROR(testName,"failed to match.");

        return( 0 );
    }

    //
    // LinearSpringConstraint
    if( testName == std::string( "LinearSpring" ) )
    {
        osg::ref_ptr< osgbDynamics::LinearSpringConstraint > cons = new osgbDynamics::LinearSpringConstraint(
            rbA, aXform, rbB, bXform, osg::Vec3( 2., 1., 0. ) );
        cons->setLimit( osg::Vec2( -2., 3. ) );
        cons->setStiffness( 40.f );
        cons->setDamping( .5f );

        if( cons->getAsBtGeneric6DofSpring() == NULL )
            ERROR(testName,"won't typecast as btGeneric6DofSpringConstraint.");

        if( !( osgDB::writeObjectFile( *cons, fileName ) ) )
            ERROR(testName,"writeObjectFile failed.");

        osg::Object* obj = osgDB::readObjectFile( fileName );
        if( obj == NULL )
            ERROR(testName,"readObjectFile returned NULL.");

        osg::ref_ptr< osgbDynamics::LinearSpringConstraint > cons2 = dynamic_cast<
            osgbDynamics::LinearSpringConstraint* >( obj );
        if( !( cons2.valid() ) )
            ERROR(testName,"dynamic_cast after readObjectFile failed.");

        if( *cons2 != *cons )
            // Note matches can fail due to double precision roundoff.
            // For testing, use only 1s and 0s in matrices.
            ERROR(testName,"failed to match.");

        return( 0 );
    }

    //
    // AngleSpringConstraint
    if( testName == std::string( "AngleSpring" ) )
    {
        osg::ref_ptr< osgbDynamics::AngleSpringConstraint > cons = new osgbDynamics::AngleSpringConstraint(
            rbA, aXform, rbB, bXform, osg::Vec3( 2., 1., 0. ), osg::Vec3( 5., 6., -7. ) );
        cons->setLimit( osg::Vec2( -2., 1. ) );
        cons->setStiffness( 50.f );
        cons->setDamping( 0.f );

        if( cons->getAsBtGeneric6DofSpring() == NULL )
            ERROR(testName,"won't typecast as btGeneric6DofSpringConstraint.");

        if( !( osgDB::writeObjectFile( *cons, fileName ) ) )
            ERROR(testName,"writeObjectFile failed.");

        osg::Object* obj = osgDB::readObjectFile( fileName );
        if( obj == NULL )
            ERROR(testName,"readObjectFile returned NULL.");

        osg::ref_ptr< osgbDynamics::AngleSpringConstraint > cons2 = dynamic_cast<
            osgbDynamics::AngleSpringConstraint* >( obj );
        if( !( cons2.valid() ) )
            ERROR(testName,"dynamic_cast after readObjectFile failed.");

        if( *cons2 != *cons )
            // Note matches can fail due to double precision roundoff.
            // For testing, use only 1s and 0s in matrices.
            ERROR(testName,"failed to match.");

        return( 0 );
    }

    //
    // LinearAngleSpringConstraint
    if( testName == std::string( "LinearAngleSpring" ) )
    {
        osg::ref_ptr< osgbDynamics::LinearAngleSpringConstraint > cons = new osgbDynamics::LinearAngleSpringConstraint(
            rbA, aXform, rbB, bXform, osg::Vec3( 2., 1., 0. ), osg::Vec3( 5., 6., -7. ) );
        cons->setLinearLimit( osg::Vec2( -2., 2. ) );
        cons->setAngleLimit( osg::Vec2( -3., 3. ) );
        cons->setLinearStiffness( 41.f );
        cons->setLinearDamping( 1.f );
        cons->setAngleStiffness( 42.f );
        cons->setAngleDamping( 2.f );

        if( cons->getAsBtGeneric6DofSpring() == NULL )
            ERROR(testName,"won't typecast as btGeneric6DofSpringConstraint.");

        if( !( osgDB::writeObjectFile( *cons, fileName ) ) )
            ERROR(testName,"writeObjectFile failed.");

        osg::Object* obj = osgDB::readObjectFile( fileName );
        if( obj == NULL )
            ERROR(testName,"readObjectFile returned NULL.");

        osg::ref_ptr< osgbDynamics::LinearAngleSpringConstraint > cons2 = dynamic_cast<
            osgbDynamics::LinearAngleSpringConstraint* >( obj );
        if( !( cons2.valid() ) )
            ERROR(testName,"dynamic_cast after readObjectFile failed.");

        if( *cons2 != *cons )
            // Note matches can fail due to double precision roundoff.
            // For testing, use only 1s and 0s in matrices.
            ERROR(testName,"failed to match.");

        return( 0 );
    }

    //
    // HingeConstraint
    if( testName == std::string( "Hinge" ) )
    {
        osg::Vec3 axis( -1., 0., 0. );
        osg::Vec3 point( 4., 3., 2. );
        osg::Vec2 limit( -2., 2. );
        osg::ref_ptr< osgbDynamics::HingeConstraint > cons = new osgbDynamics::HingeConstraint(
            rbA, aXform, rbB, bXform, axis, point, limit );

        if( cons->getAsBtHinge() == NULL )
            ERROR(testName,"won't typecast as btHingeConstraint.");

        if( !( osgDB::writeObjectFile( *cons, fileName ) ) )
            ERROR(testName,"writeObjectFile failed.");

        osg::Object* obj = osgDB::readObjectFile( fileName );
        if( obj == NULL )
            ERROR(testName,"readObjectFile returned NULL.");

        osg::ref_ptr< osgbDynamics::HingeConstraint > cons2 = dynamic_cast<
            osgbDynamics::HingeConstraint* >( obj );
        if( !( cons2.valid() ) )
            ERROR(testName,"dynamic_cast after readObjectFile failed.");

        if( *cons2 != *cons )
            // Note matches can fail due to double precision roundoff.
            // For testing, use only 1s and 0s in matrices.
            ERROR(testName,"failed to match.");

        return( 0 );
    }

    //
    // CardanConstraint
    if( testName == std::string( "Cardan" ) )
    {
        osg::Vec3 axisA( -1., 0., 0. );
        osg::Vec3 axisB( 0., 0., 1. );
        osg::ref_ptr< osgbDynamics::CardanConstraint > cons = new osgbDynamics::CardanConstraint(
            rbA, aXform, rbB, bXform, axisA, axisB );

        if( cons->getAsBtUniversal() == NULL )
            ERROR(testName,"won't typecast as btUniversalConstraint.");

        if( !( osgDB::writeObjectFile( *cons, fileName ) ) )
            ERROR(testName,"writeObjectFile failed.");

        osg::Object* obj = osgDB::readObjectFile( fileName );
        if( obj == NULL )
            ERROR(testName,"readObjectFile returned NULL.");

        osg::ref_ptr< osgbDynamics::CardanConstraint > cons2 = dynamic_cast<
            osgbDynamics::CardanConstraint* >( obj );
        if( !( cons2.valid() ) )
            ERROR(testName,"dynamic_cast after readObjectFile failed.");

        if( *cons2 != *cons )
            // Note matches can fail due to double precision roundoff.
            // For testing, use only 1s and 0s in matrices.
            ERROR(testName,"failed to match.");

        return( 0 );
    }

    //
    // RagdollConstraint
    if( testName == std::string( "Ragdoll" ) )
    {
        osg::Vec3 point( 0., 1., 2. );
        osg::Vec3 axis( 0., 1., 0. );
        double angle = 2.;
        osg::ref_ptr< osgbDynamics::RagdollConstraint > cons = new osgbDynamics::RagdollConstraint(
            rbA, aXform, rbB, bXform, point, axis, angle );

        if( cons->getAsBtConeTwist() == NULL )
            ERROR(testName,"won't typecast as btConeTwistConstraint.");

        if( !( osgDB::writeObjectFile( *cons, fileName ) ) )
            ERROR(testName,"writeObjectFile failed.");

        osg::Object* obj = osgDB::readObjectFile( fileName );
        if( obj == NULL )
            ERROR(testName,"readObjectFile returned NULL.");

        osg::ref_ptr< osgbDynamics::RagdollConstraint > cons2 = dynamic_cast<
            osgbDynamics::RagdollConstraint* >( obj );
        if( !( cons2.valid() ) )
            ERROR(testName,"dynamic_cast after readObjectFile failed.");

        if( *cons2 != *cons )
            // Note matches can fail due to double precision roundoff.
            // For testing, use only 1s and 0s in matrices.
            ERROR(testName,"failed to match.");

        return( 0 );
    }

    //
    // WheelSuspensionConstraint
    if( testName == std::string( "WheelSuspension" ) )
    {
        osg::Vec3 springAxis( 0., 0., 1. );
        osg::Vec3 axleAxis( 0., 1., 0. );
        osg::Vec2 linearLimit( -2., 3. );
        osg::Vec2 angleLimit( -1., 1. );
        osg::Vec3 anchor( 0., 1., 2. );

        osg::ref_ptr< osgbDynamics::WheelSuspensionConstraint > cons = new osgbDynamics::WheelSuspensionConstraint(
            rbA, rbB, springAxis, axleAxis, linearLimit, angleLimit, anchor );

        if( cons->getAsBtHinge2() == NULL )
            ERROR(testName,"won't typecast as btHinge2Constraint.");

        if( !( osgDB::writeObjectFile( *cons, fileName ) ) )
            ERROR(testName,"writeObjectFile failed.");

        osg::Object* obj = osgDB::readObjectFile( fileName );
        if( obj == NULL )
            ERROR(testName,"readObjectFile returned NULL.");

        osg::ref_ptr< osgbDynamics::WheelSuspensionConstraint > cons2 = dynamic_cast<
            osgbDynamics::WheelSuspensionConstraint* >( obj );
        if( !( cons2.valid() ) )
            ERROR(testName,"dynamic_cast after readObjectFile failed.");

        if( *cons2 != *cons )
            // Note matches can fail due to double precision roundoff.
            // For testing, use only 1s and 0s in matrices.
            ERROR(testName,"failed to match.");

        return( 0 );
    }

    ERROR(testName,"unknown test name.");
}
