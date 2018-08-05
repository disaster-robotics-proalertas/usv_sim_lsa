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

#include <BulletDynamics/ConstraintSolver/btContactConstraint.h>
#include <osgbDynamics/Constraints.h>
#include <osgbDynamics/MotionState.h>
#include <osgbCollision/Utils.h>

#include <osg/Object>
#include <osg/Notify>

#include <btBulletDynamicsCommon.h>

#include <osg/io_utils>


namespace osgbDynamics
{



Constraint::Constraint()
  : osg::Object(),
    _constraint( NULL ),
    _dirty( true ),
    _rbA( NULL ),
    _rbB( NULL )
{
}

Constraint::Constraint( btRigidBody* rbA, btRigidBody* rbB )
  : osg::Object(),
    _constraint( NULL ),
    _dirty( true ),
    _rbA( rbA ),
    _rbB( rbB )
{
}

Constraint::Constraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform )
  : _constraint( NULL ),
    _dirty( true ),
    _rbA( rbA ),
    _rbB( rbB ),
    _rbAXform( rbAXform ),
    _rbBXform( rbBXform )
{
}

Constraint::Constraint( const Constraint& rhs, const osg::CopyOp& copyop )
  : osg::Object( rhs, copyop ),
    _constraint( rhs._constraint ),
    _dirty( rhs._dirty ),
    _rbA( rhs._rbA ),
    _rbB( rhs._rbB ),
    _rbAXform( rhs._rbAXform ),
    _rbBXform( rhs._rbBXform )
{
}

Constraint::~Constraint()
{
    // Deleting the constraint is up to the calling code. Something like this:
    //delete osgbDynamics::Constraint::getConstraint();
}

btTypedConstraint* Constraint::getConstraint() const
{
    if( getDirty() || ( _constraint == NULL ) )
    {
        Constraint* nonConst = const_cast< Constraint* >( this );
        nonConst->createConstraint();
    }

    return( _constraint );
}

void Constraint::setRigidBodies( btRigidBody* rbA, btRigidBody* rbB )
{
    _rbA = rbA;
    _rbB = rbB;
    setDirty();
}
void Constraint::setAXform( const osg::Matrix& rbAXform )
{
    _rbAXform = rbAXform;
    setDirty();
}
void Constraint::setBXform( const osg::Matrix& rbBXform )
{
    _rbBXform = rbBXform;
    setDirty();
}

bool Constraint::operator==( const Constraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool Constraint::operator!=( const Constraint& rhs ) const
{
    return(
        ( _rbAXform != rhs._rbAXform ) ||
        ( _rbBXform != rhs._rbBXform )
    );
}

osg::Matrix Constraint::orthonormalize( const osg::Matrix& in )
{
    osg::Vec3d a( in( 0, 0 ), in( 0, 1 ), in( 0, 2 ) );
    osg::Vec3d b( in( 1, 0 ), in( 1, 1 ), in( 1, 2 ) );
    osg::Vec3d c( a ^ b );
    c.normalize();
    b = c ^ a;
    b.normalize();
    a = b ^ c;
    a.normalize();

    osg::Matrix m( a[0], a[1], a[2], in(0,3),
        b[0], b[1], b[2], in(1,3),
        c[0], c[1], c[2], in(2,3),
        in(3,0), in(3,1), in(3,2), in(3,3) );
    return( m );
}




SliderConstraint::SliderConstraint()
  : Constraint(),
    _axis( 1., 0., 0. ),
    _slideLimit( -1., 1. )
{
}
SliderConstraint::SliderConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : Constraint( rbA, rbB ),
    _axis( 1., 0., 0. ),
    _slideLimit( -1., 1. )
{
    setDirty();
}
SliderConstraint::SliderConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        const osg::Vec3& axis, const osg::Vec2& slideLimit )
  : Constraint( rbA, rbAXform ),
    _axis( axis ),
    _slideLimit( slideLimit )
{
    setDirty();
}
SliderConstraint::SliderConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform,
        const osg::Vec3& axis, const osg::Vec2& slideLimit )
  : Constraint( rbA, rbAXform, rbB, rbBXform ),
    _axis( axis ),
    _slideLimit( slideLimit )
{
    setDirty();
}
SliderConstraint::SliderConstraint( const SliderConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _axis( rhs._axis),
    _slideLimit( rhs._slideLimit )
{
}
SliderConstraint::~SliderConstraint()
{
    // Deleting the constraint is up to the calling code.
}

btSliderConstraint* SliderConstraint::getAsBtSlider() const
{
    return( static_cast< btSliderConstraint* >( getConstraint() ) );
}

void SliderConstraint::setAxis( const osg::Vec3& axis )
{
    _axis = axis;
    setDirty();
}
void SliderConstraint::setAxis( const double x, const double y, const double z )
{
    setAxis( osg::Vec3( x, y, z ) );
}
void SliderConstraint::setLimit( const osg::Vec2& limit )
{
    _slideLimit = limit;

    if( !getDirty() && ( _constraint != NULL ) )
    {
        // Dynamically modify the existing constraint.
        btSliderConstraint* cons = getAsBtSlider();
        cons->setLowerLinLimit( _slideLimit[ 0 ] );
        cons->setUpperLinLimit( _slideLimit[ 1 ] );
    }
    else
        setDirty();
}
void SliderConstraint::setLimit( const double lo, const double hi )
{
    setLimit( osg::Vec2( lo, hi ) );
}

bool SliderConstraint::operator==( const SliderConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool SliderConstraint::operator!=( const SliderConstraint& rhs ) const
{
    return(
        ( _axis != rhs._axis ) ||
        ( _slideLimit != rhs._slideLimit ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

osg::Matrix on( const osg::Matrix& in )
{
    osg::Vec3d a( in( 0, 0 ), in( 0, 1 ), in( 0, 2 ) );
    osg::Vec3d b( in( 1, 0 ), in( 1, 1 ), in( 1, 2 ) );
    osg::Vec3d c( a ^ b );
    c.normalize();
    b = c ^ a;
    b.normalize();
    a = b ^ c;
    a.normalize();

    osg::Matrix m( a[0], a[1], a[2], in(0,3),
        b[0], b[1], b[2], in(1,3),
        c[0], c[1], c[2], in(2,3),
        in(3,0), in(3,1), in(3,2), in(3,3) );
    return( m );
}

void SliderConstraint::createConstraint()
{
    if( _rbA == NULL )
    {
        osg::notify( osg::INFO ) << "createConstraint: _rbA == NULL." << std::endl;
        return;
    }

    if( _constraint != NULL )
    {
        delete _constraint;
        _constraint = NULL;
    }


    // Orientation matrix for the slider x-axis.
    osg::Vec3 localAxis( _axis );
    localAxis.normalize();
    const osg::Matrix orientation = osg::Matrix::rotate( osg::Vec3( 1., 0., 0. ), localAxis );


    // Create a matrix that returns A to the origin.
    //
    //   1. Inverse A center of mass offset.
    osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( _rbA->getMotionState() );
    if( motion == NULL )
    {
        osg::notify( osg::WARN ) << "createConstraint: Invalid MotionState." << std::endl;
        return;
    }
    osg::Vec3 invCom = -( motion->getCenterOfMass() );
    osg::Vec3 scale = motion->getScale();
    osg::Vec3 scaledInvCom = osg::Vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
    const osg::Matrix invACOM( osg::Matrix::translate( scaledInvCom ) );
    //
    //   2. Transform A back to the origin.
    const osg::Matrix invAXform( osg::Matrix::inverse( orthonormalize( _rbAXform ) ) );
    //
    //   3. The final rbA frame matrix.
    btTransform rbAFrame = osgbCollision::asBtTransform( 
        orientation * invAXform * invACOM );


    btTransform rbBFrame;
    if( _rbB != NULL )
    {
        // Create a matrix that orients the spring axis/point in B's coordinate space.
        //
        //   1. Inverse B center of mass offset.
        motion = dynamic_cast< osgbDynamics::MotionState* >( _rbB->getMotionState() );
        if( motion == NULL )
        {
            osg::notify( osg::WARN ) << "InternalCreateSpring: Invalid MotionState." << std::endl;
            return;
        }
        invCom = -( motion->getCenterOfMass() );
        scale = motion->getScale();
        scaledInvCom = osg::Vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
        const osg::Matrix invBCOM( osg::Matrix::translate( scaledInvCom ) );
        //
        //   2. Transform B back to the origin.
        const osg::Matrix invBXform( osg::Matrix::inverse( orthonormalize( _rbBXform ) ) );
        //
        //   3. The final rbB frame matrix.
        rbBFrame = osgbCollision::asBtTransform( 
            orientation * invBXform * invBCOM );
    }


    btSliderConstraint* cons;
    if( _rbB != NULL )
        cons = new btSliderConstraint( *_rbA, *_rbB, rbAFrame, rbBFrame, false );
    else
        cons = new btSliderConstraint( *_rbA, rbAFrame, true );

    cons->setLowerLinLimit( btScalar( _slideLimit[ 0 ] ) );
    cons->setUpperLinLimit( btScalar( _slideLimit[ 1 ] ) );

    _constraint = cons;

    setDirty( false );
}




TwistSliderConstraint::TwistSliderConstraint()
  : Constraint(),
    _axis( 1., 0., 0. ),
    _point( 0., 0., 0. ),
    _slideLimit( -1., 1. ),
    _twistLimit( -osg::PI_2, osg::PI_2 )
{
}
TwistSliderConstraint::TwistSliderConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : Constraint( rbA, rbB ),
    _axis( 1., 0., 0. ),
    _point( 0., 0., 0. ),
    _slideLimit( -1., 1. ),
    _twistLimit( -osg::PI_2, osg::PI_2 )
{
}
TwistSliderConstraint::TwistSliderConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB, const osg::Matrix& rbBXform,
            const osg::Vec3& axis, const osg::Vec3& wcPoint,
            const osg::Vec2& slideLimit, const osg::Vec2& twistLimit )
  : Constraint( rbA, rbAXform, rbB, rbBXform ),
    _axis( axis ),
    _point( wcPoint ),
    _slideLimit( slideLimit ),
    _twistLimit( twistLimit )
{
}
TwistSliderConstraint::TwistSliderConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            const osg::Vec3& axis, const osg::Vec3& wcPoint,
            const osg::Vec2& slideLimit, const osg::Vec2& twistLimit )
  : Constraint( rbA, rbAXform ),
    _axis( axis ),
    _point( wcPoint ),
    _slideLimit( slideLimit ),
    _twistLimit( twistLimit )
{
}
TwistSliderConstraint::TwistSliderConstraint( const TwistSliderConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _axis( rhs._axis ),
    _point( rhs._point ),
    _slideLimit( rhs._slideLimit ),
    _twistLimit( rhs._twistLimit )
{
}
TwistSliderConstraint::~TwistSliderConstraint()
{
}

btSliderConstraint* TwistSliderConstraint::getAsBtSlider() const
{
    return( static_cast< btSliderConstraint* >( getConstraint() ) );
}

void TwistSliderConstraint::setAxis( const osg::Vec3& axis )
{
    _axis = axis;
    setDirty();
}
void TwistSliderConstraint::setAxis( const double x, const double y, const double z )
{
    setAxis( osg::Vec3( x, y, z ) );
}
void TwistSliderConstraint::setPoint( const osg::Vec3& wcPoint )
{
    _point = wcPoint;
    setDirty();
}
void TwistSliderConstraint::setPoint( const double x, const double y, const double z )
{
    setPoint( osg::Vec3( x, y, z ) );
}
void TwistSliderConstraint::setSlideLimit( const osg::Vec2& limit )
{
    _slideLimit = limit;

    if( !getDirty() && ( _constraint != NULL ) )
    {
        // Dynamically modify the existing constraint.
        btSliderConstraint* cons = getAsBtSlider();
        cons->setLowerLinLimit( _slideLimit[ 0 ] );
        cons->setUpperLinLimit( _slideLimit[ 1 ] );
    }
    else
        setDirty();
}
void TwistSliderConstraint::setSlideLimit( const double lo, const double hi )
{
    setSlideLimit( osg::Vec2( lo, hi ) );
}
void TwistSliderConstraint::setTwistLimit( const osg::Vec2& limitRadians )
{
    _twistLimit = limitRadians;

    if( !getDirty() && ( _constraint != NULL ) )
    {
        // Dynamically modify the existing constraint.
        btSliderConstraint* cons = getAsBtSlider();
        cons->setLowerAngLimit( _twistLimit[ 0 ] );
        cons->setUpperAngLimit( _twistLimit[ 1 ] );
    }
    else
        setDirty();
}
void TwistSliderConstraint::setTwistLimit( const double lo, const double hi )
{
    setTwistLimit( osg::Vec2( lo, hi ) );
}

bool TwistSliderConstraint::operator==( const TwistSliderConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool TwistSliderConstraint::operator!=( const TwistSliderConstraint& rhs ) const
{
    return(
        ( _axis != rhs._axis ) ||
        ( _point != rhs._point ) ||
        ( _slideLimit != rhs._slideLimit ) ||
        ( _twistLimit != rhs._twistLimit ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}


void TwistSliderConstraint::createConstraint()
{
    if( _rbA == NULL )
    {
        osg::notify( osg::INFO ) << "createConstraint: _rbA == NULL." << std::endl;
        return;
    }

    if( _constraint != NULL )
    {
        delete _constraint;
        _constraint = NULL;
    }


    // Orientation matrix for the slider x-axis / point.
    osg::Vec3 localAxis( _axis );
    localAxis.normalize();
    const osg::Matrix orientation =
        osg::Matrix::rotate( osg::Vec3( 1., 0., 0. ), localAxis ) *
        osg::Matrix::translate( _point );


    // Create a matrix that returns A to the origin.
    //
    //   1. Inverse A center of mass offset.
    osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( _rbA->getMotionState() );
    if( motion == NULL )
    {
        osg::notify( osg::WARN ) << "createConstraint: Invalid MotionState." << std::endl;
        return;
    }
    osg::Vec3 invCom = -( motion->getCenterOfMass() );
    osg::Vec3 scale = motion->getScale();
    osg::Vec3 scaledInvCom = osg::Vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
    const osg::Matrix invACOM( osg::Matrix::translate( scaledInvCom ) );
    //
    //   2. Transform A back to the origin.
    const osg::Matrix invAXform( osg::Matrix::inverse( orthonormalize( _rbAXform ) ) );
    //
    //   3. The final rbA frame matrix.
    btTransform rbAFrame = osgbCollision::asBtTransform( 
        orientation * invAXform * invACOM );


    btTransform rbBFrame;
    if( _rbB != NULL )
    {
        // Create a matrix that orients the spring axis/point in B's coordinate space.
        //
        //   1. Inverse B center of mass offset.
        motion = dynamic_cast< osgbDynamics::MotionState* >( _rbB->getMotionState() );
        if( motion == NULL )
        {
            osg::notify( osg::WARN ) << "InternalCreateSpring: Invalid MotionState." << std::endl;
            return;
        }
        invCom = -( motion->getCenterOfMass() );
        scale = motion->getScale();
        scaledInvCom = osg::Vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
        const osg::Matrix invBCOM( osg::Matrix::translate( scaledInvCom ) );
        //
        //   2. Transform B back to the origin.
        const osg::Matrix invBXform( osg::Matrix::inverse( orthonormalize( _rbBXform ) ) );
        //
        //   3. The final rbB frame matrix.
        rbBFrame = osgbCollision::asBtTransform( 
            orientation * invBXform * invBCOM );
    }


    btSliderConstraint* cons;
    if( _rbB != NULL )
        cons = new btSliderConstraint( *_rbA, *_rbB, rbAFrame, rbBFrame, false );
    else
        cons = new btSliderConstraint( *_rbA, rbAFrame, true );

    cons->setLowerLinLimit( btScalar( _slideLimit[ 0 ] ) );
    cons->setUpperLinLimit( btScalar( _slideLimit[ 1 ] ) );
    cons->setLowerAngLimit( _twistLimit[ 0 ] );
    cons->setUpperAngLimit( _twistLimit[ 1 ] );

    _constraint = cons;

    setDirty( false );
}




InternalSpringData::InternalSpringData()
  : _linearLowerLimits( 0., 0., 0. ),
    _linearUpperLimits( 0., 0., 0. ),
    _angularLowerLimits( 0., 0., 0. ),
    _angularUpperLimits( 0., 0., 0. )
{
    for( int idx=0; idx<6; idx++ )
    {
        _enable[ idx ] = false;
        _stiffness[ idx ] = _damping[ idx ] = 0.;
    }
}
InternalSpringData::InternalSpringData( const InternalSpringData& rhs, const osg::CopyOp& copyop )
  : _linearLowerLimits( rhs._linearLowerLimits ),
    _linearUpperLimits( rhs._linearUpperLimits ),
    _angularLowerLimits( rhs._angularLowerLimits ),
    _angularUpperLimits( rhs._angularUpperLimits )
{
    memcpy( _enable, rhs._enable, sizeof( _enable ) );
    memcpy( _stiffness, rhs._stiffness, sizeof( _stiffness ) );
    memcpy( _damping, rhs._damping, sizeof( _damping ) );
}

void InternalSpringData::apply( btGeneric6DofSpringConstraint* cons ) const
{
    cons->setLinearLowerLimit( osgbCollision::asBtVector3( _linearLowerLimits ) );
    cons->setLinearUpperLimit( osgbCollision::asBtVector3( _linearUpperLimits ) );
    cons->setAngularLowerLimit( osgbCollision::asBtVector3( _angularLowerLimits ) );
    cons->setAngularUpperLimit( osgbCollision::asBtVector3( _angularUpperLimits ) );

    int idx;
    for( idx=0; idx<6; idx++ )
    {
        cons->enableSpring( idx, _enable[ idx ] );
        cons->setStiffness( idx, _stiffness[ idx ] );
        cons->setDamping( idx, _damping[ idx ] );
    }
}

bool InternalSpringData::operator==( const InternalSpringData& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool InternalSpringData::operator!=( const InternalSpringData& rhs ) const
{
    if( ( _linearLowerLimits != rhs._linearLowerLimits ) ||
        ( _linearUpperLimits != rhs._linearUpperLimits ) ||
        ( _angularLowerLimits != rhs._angularLowerLimits ) ||
        ( _angularUpperLimits != rhs._angularUpperLimits ) )
        return( true );

    int idx;
    for( idx=0; idx<6; idx++ )
    {
        if( ( _enable[ idx ] != rhs._enable[ idx ] ) ||
            ( _stiffness[ idx ] != rhs._stiffness[ idx ] ) ||
            ( _damping[ idx ] != rhs._damping[ idx ] ) )
            return( true );
    }

    return( false );
}




LinearSpringConstraint::LinearSpringConstraint()
  : Constraint(),
    _data( new InternalSpringData )
{
    _data->_enable[ 0 ] = true;
    _data->_linearLowerLimits[ 0 ] = -1.;
    _data->_linearUpperLimits[ 0 ] = 1.;
    _data->_stiffness[ 0 ] = 10.f;
    _data->_damping[ 0 ] = .1f;
}
LinearSpringConstraint::LinearSpringConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : Constraint( rbA, rbB ),
    _axis( 1., 0., 0. ),
    _data( new InternalSpringData )
{
    _data->_enable[ 0 ] = true;
    _data->_linearLowerLimits[ 0 ] = -1.;
    _data->_linearUpperLimits[ 0 ] = 1.;
    _data->_stiffness[ 0 ] = 10.f;
    _data->_damping[ 0 ] = .1f;
}
LinearSpringConstraint::LinearSpringConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform, const osg::Vec3& axis )
  : Constraint( rbA, rbAXform, rbB, rbBXform ),
    _axis( axis ),
    _data( new InternalSpringData )
{
    _data->_enable[ 0 ] = true;
    _data->_linearLowerLimits[ 0 ] = -1.;
    _data->_linearUpperLimits[ 0 ] = 1.;
    _data->_stiffness[ 0 ] = 10.f;
    _data->_damping[ 0 ] = .1f;
}
LinearSpringConstraint::LinearSpringConstraint( const LinearSpringConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _axis( rhs._axis ),
    _data( rhs._data )
{
}
LinearSpringConstraint::~LinearSpringConstraint()
{
}

btGeneric6DofSpringConstraint* LinearSpringConstraint::getAsBtGeneric6DofSpring() const
{
    return( static_cast< btGeneric6DofSpringConstraint* >( getConstraint() ) );
}

void LinearSpringConstraint::setSpringData( InternalSpringData* data )
{
    _data = data;

    if( !getDirty() && ( _constraint != NULL ) )
    {
        // Dynamically modify the existing constraint.
        btGeneric6DofSpringConstraint* cons = getAsBtGeneric6DofSpring();
        _data->apply( cons );
    }
    else
        setDirty();
}
void LinearSpringConstraint::setAxis( const osg::Vec3& axis )
{
    _axis = axis;
    setDirty();
}
void LinearSpringConstraint::setAxis( const double x, const double y, const double z )
{
    setAxis( osg::Vec3( x, y, z ) );
}
void LinearSpringConstraint::setLimit( const osg::Vec2& limit )
{
    _data->_linearLowerLimits[ 0 ] = limit[ 0 ];
    _data->_linearUpperLimits[ 0 ] = limit[ 1 ];
    setSpringData( _data.get() );
}
void LinearSpringConstraint::setLimit( const double lo, const double hi )
{
    LinearSpringConstraint::setLimit( osg::Vec2( lo, hi ) );
}
void LinearSpringConstraint::setStiffness( float stiffness )
{
    _data->_stiffness[ 0 ] = btScalar( stiffness );
    setSpringData( _data.get() );
}
void LinearSpringConstraint::setDamping( float damping )
{
    _data->_damping[ 0 ] = btScalar( damping );
    setSpringData( _data.get() );
}

bool LinearSpringConstraint::operator==( const LinearSpringConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool LinearSpringConstraint::operator!=( const LinearSpringConstraint& rhs ) const
{
    return(
        ( _axis != rhs._axis ) ||
        ( *_data != *( rhs._data ) ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void LinearSpringConstraint::createConstraint()
{
    if( _constraint != NULL )
    {
        delete _constraint;
        _constraint = NULL;
    }

    // The internal construction/config code for Bullet spring constraints
    // is identical for our three rpring types, except that LinearSpringConstraint
    // does not have a pivot point.
    _constraint = internalCreateSpringConstraint( this, _data.get(), _axis );

    setDirty( _constraint == NULL );
}

btGeneric6DofSpringConstraint* LinearSpringConstraint::internalCreateSpringConstraint(
    Constraint* cons, const InternalSpringData* isd,
    const osg::Vec3& axis, const osg::Vec3& point )
{
    btRigidBody* rbA, * rbB;
    cons->getRigidBodies( rbA, rbB );

    if( ( rbA == NULL ) || ( rbB == NULL ) )
    {
        osg::notify( osg::INFO ) << "InternalSpringCreate: _rbA == NULL or _rbB == NULL." << std::endl;
        return( NULL );
    }

    const osg::Matrix aXform = cons->getAXform();
    const osg::Matrix bXform = cons->getBXform();


    // Orientation matrix for the spring x-axis / point.
    osg::Vec3 localAxis( axis );
    localAxis.normalize();
    const osg::Matrix orientation =
        osg::Matrix::rotate( osg::Vec3( 1., 0., 0. ), localAxis ) *
        osg::Matrix::translate( point );


    // Create a matrix that puts A in B's coordinate space, accounting
    // for orientation of the constraint axes.
    //
    //   1. Inverse A center of mass offset.
    osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( rbA->getMotionState() );
    if( motion == NULL )
    {
        osg::notify( osg::WARN ) << "InternalCreateSpring: Invalid MotionState." << std::endl;
        return( NULL );
    }
    osg::Vec3 invCom = -( motion->getCenterOfMass() );
    osg::Vec3 scale = motion->getScale();
    osg::Vec3 scaledInvCom = osg::Vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
    const osg::Matrix invACOM( osg::Matrix::translate( scaledInvCom ) );
    //
    //   2. Transform A back to the origin.
    const osg::Matrix invAXform( osg::Matrix::inverse( aXform ) );
    //
    //   3. The final rbA frame matrix.
    btTransform rbAFrame = osgbCollision::asBtTransform( 
        orientation * invAXform * invACOM );


    // Create a matrix that orients the spring axis/point in B's coordinate space.
    //
    //   1. Inverse B center of mass offset.
    motion = dynamic_cast< osgbDynamics::MotionState* >( rbB->getMotionState() );
    if( motion == NULL )
    {
        osg::notify( osg::WARN ) << "InternalCreateSpring: Invalid MotionState." << std::endl;
        return( NULL );
    }
    invCom = -( motion->getCenterOfMass() );
    scale = motion->getScale();
    scaledInvCom = osg::Vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
    const osg::Matrix invBCOM( osg::Matrix::translate( scaledInvCom ) );
    //
    //   2. Transform B back to the origin.
    const osg::Matrix invBXform( osg::Matrix::inverse( bXform ) );
    //
    //   3. The final rbB frame matrix.
    btTransform rbBFrame = osgbCollision::asBtTransform( 
        orientation * invBXform * invBCOM );


    btGeneric6DofSpringConstraint* springCons = new btGeneric6DofSpringConstraint(
        *rbA, *rbB, rbAFrame, rbBFrame, false );
    isd->apply( springCons );
    springCons->setEquilibriumPoint();

    return( springCons );
}




AngleSpringConstraint::AngleSpringConstraint()
  : Constraint(),
    _axis( 1., 0., 0. ),
    _pivotPoint( 0., 0., 0. ),
    _data( new InternalSpringData )
{
    _data->_enable[ 3 ] = true;
    _data->_angularLowerLimits[ 0 ] = -osg::PI_2;
    _data->_angularUpperLimits[ 0 ] = osg::PI_2;
    _data->_stiffness[ 3 ] = 10.f;
    _data->_damping[ 3 ] = .1f;
}
AngleSpringConstraint::AngleSpringConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : Constraint( rbA, rbB ),
    _axis( 1., 0., 0. ),
    _pivotPoint( 0., 0., 0. ),
    _data( new InternalSpringData )
{
    _data->_enable[ 3 ] = true;
    _data->_angularLowerLimits[ 0 ] = -osg::PI_2;
    _data->_angularUpperLimits[ 0 ] = osg::PI_2;
    _data->_stiffness[ 3 ] = 10.f;
    _data->_damping[ 3 ] = .1f;
}
AngleSpringConstraint::AngleSpringConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform,
        const osg::Vec3& axis, const osg::Vec3& point )
  : Constraint( rbA, rbAXform, rbB, rbBXform ),
    _axis( axis ),
    _pivotPoint( point ),
    _data( new InternalSpringData )
{
    _data->_enable[ 3 ] = true;
    _data->_angularLowerLimits[ 0 ] = -osg::PI_2;
    _data->_angularUpperLimits[ 0 ] = osg::PI_2;
    _data->_stiffness[ 3 ] = 10.f;
    _data->_damping[ 3 ] = .1f;
}
AngleSpringConstraint::AngleSpringConstraint( const AngleSpringConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _axis( rhs._axis ),
    _pivotPoint( rhs._pivotPoint ),
    _data( rhs._data )
{
}
AngleSpringConstraint::~AngleSpringConstraint()
{
}

btGeneric6DofSpringConstraint* AngleSpringConstraint::getAsBtGeneric6DofSpring() const
{
    return( static_cast< btGeneric6DofSpringConstraint* >( getConstraint() ) );
}

void AngleSpringConstraint::setSpringData( InternalSpringData* data )
{
    _data = data;

    if( !getDirty() && ( _constraint != NULL ) )
    {
        // Dynamically modify the existing constraint.
        btGeneric6DofSpringConstraint* cons = getAsBtGeneric6DofSpring();
        _data->apply( cons );
    }
    else
        setDirty();
}
void AngleSpringConstraint::setAxis( const osg::Vec3& axis )
{
    _axis = axis;
    setDirty();
}
void AngleSpringConstraint::setAxis( const double x, const double y, const double z )
{
    setAxis( osg::Vec3( x, y, z ) );
}
void AngleSpringConstraint::setPivotPoint( const osg::Vec3& wcPoint )
{
    _pivotPoint = wcPoint;
    setDirty();
}
void AngleSpringConstraint::setPivotPoint( const double x, const double y, const double z )
{
    setPivotPoint( osg::Vec3( x, y, z ) );
}
void AngleSpringConstraint::setLimit( const osg::Vec2& limit )
{
    _data->_angularLowerLimits[ 0 ] = limit[ 0 ];
    _data->_angularUpperLimits[ 0 ] = limit[ 1 ];
    setSpringData( _data.get() );
}
void AngleSpringConstraint::setLimit( const double lo, const double hi )
{
    setLimit( osg::Vec2( lo, hi ) );
}
void AngleSpringConstraint::setStiffness( float stiffness )
{
    _data->_stiffness[ 3 ] = btScalar( stiffness );
    setSpringData( _data.get() );
}
void AngleSpringConstraint::setDamping( float damping )
{
    _data->_damping[ 3 ] = btScalar( damping );
    setSpringData( _data.get() );
}

bool AngleSpringConstraint::operator==( const AngleSpringConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool AngleSpringConstraint::operator!=( const AngleSpringConstraint& rhs ) const
{
    return(
        ( _axis != rhs._axis ) ||
        ( *_data != *( rhs._data ) ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void AngleSpringConstraint::createConstraint()
{
    if( _constraint != NULL )
    {
        delete _constraint;
        _constraint = NULL;
    }

    _constraint = LinearSpringConstraint::internalCreateSpringConstraint(
        this, _data.get(), _axis, _pivotPoint );

    setDirty( _constraint == NULL );
}




LinearAngleSpringConstraint::LinearAngleSpringConstraint()
  : Constraint(),
    _axis( 1., 0., 0. ),
    _pivotPoint( 0., 0., 0. ),
    _data( new InternalSpringData )
{
    _data->_enable[ 0 ] = _data->_enable[ 3 ] = true;
    _data->_linearLowerLimits[ 0 ] = -1.;
    _data->_linearUpperLimits[ 0 ] = 1.;
    _data->_angularLowerLimits[ 0 ] = -osg::PI_2;
    _data->_angularUpperLimits[ 0 ] = osg::PI_2;
    _data->_stiffness[ 0 ] = _data->_stiffness[ 3 ] = 10.f;
    _data->_damping[ 0 ] = _data->_damping[ 3 ] = .1f;
}
LinearAngleSpringConstraint::LinearAngleSpringConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : Constraint( rbA, rbB ),
    _axis( 1., 0., 0. ),
    _pivotPoint( 0., 0., 0. ),
    _data( new InternalSpringData )
{
    _data->_enable[ 0 ] = _data->_enable[ 3 ] = true;
    _data->_linearLowerLimits[ 0 ] = -1.;
    _data->_linearUpperLimits[ 0 ] = 1.;
    _data->_angularLowerLimits[ 0 ] = -osg::PI_2;
    _data->_angularUpperLimits[ 0 ] = osg::PI_2;
    _data->_stiffness[ 0 ] = _data->_stiffness[ 3 ] = 10.f;
    _data->_damping[ 0 ] = _data->_damping[ 3 ] = .1f;
}
LinearAngleSpringConstraint::LinearAngleSpringConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform,
        const osg::Vec3& axis, const osg::Vec3& point )
  : Constraint( rbA, rbAXform, rbB, rbBXform ),
    _axis( axis ),
    _pivotPoint( point ),
    _data( new InternalSpringData )
{
    _data->_enable[ 0 ] = _data->_enable[ 3 ] = true;
    _data->_linearLowerLimits[ 0 ] = -1.;
    _data->_linearUpperLimits[ 0 ] = 1.;
    _data->_angularLowerLimits[ 0 ] = -osg::PI_2;
    _data->_angularUpperLimits[ 0 ] = osg::PI_2;
    _data->_stiffness[ 0 ] = _data->_stiffness[ 3 ] = 10.f;
    _data->_damping[ 0 ] = _data->_damping[ 3 ] = .1f;
}
LinearAngleSpringConstraint::LinearAngleSpringConstraint( const LinearAngleSpringConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _axis( rhs._axis ),
    _pivotPoint( rhs._pivotPoint ),
    _data( rhs._data )
{
}
LinearAngleSpringConstraint::~LinearAngleSpringConstraint()
{
}

btGeneric6DofSpringConstraint* LinearAngleSpringConstraint::getAsBtGeneric6DofSpring() const
{
    return( static_cast< btGeneric6DofSpringConstraint* >( getConstraint() ) );
}

void LinearAngleSpringConstraint::setSpringData( InternalSpringData* data )
{
    _data = data;

    if( !getDirty() && ( _constraint != NULL ) )
    {
        // Dynamically modify the existing constraint.
        btGeneric6DofSpringConstraint* cons = getAsBtGeneric6DofSpring();
        _data->apply( cons );
    }
    else
        setDirty();
}
void LinearAngleSpringConstraint::setAxis( const osg::Vec3& axis )
{
    _axis = axis;
    setDirty();
}
void LinearAngleSpringConstraint::setAxis( const double x, const double y, const double z )
{
    setAxis( osg::Vec3d( 2, y, z ) );
}
void LinearAngleSpringConstraint::setPivotPoint( const osg::Vec3& wcPoint )
{
    _pivotPoint = wcPoint;
    setDirty();
}
void LinearAngleSpringConstraint::setPivotPoint( const double x, const double y, const double z )
{
    setPivotPoint( osg::Vec3( x, y, z ) );
}
void LinearAngleSpringConstraint::setLinearLimit( const osg::Vec2& limit )
{
    _data->_linearLowerLimits[ 0 ] = limit[ 0 ];
    _data->_linearUpperLimits[ 0 ] = limit[ 1 ];
    setSpringData( _data.get() );
}
void LinearAngleSpringConstraint::setLinearLimit( const double lo, const double hi )
{
    setLinearLimit( osg::Vec2( lo, hi ) );
}
void LinearAngleSpringConstraint::setAngleLimit( const osg::Vec2& limit )
{
    _data->_angularLowerLimits[ 0 ] = limit[ 0 ];
    _data->_angularUpperLimits[ 0 ] = limit[ 1 ];
    setSpringData( _data.get() );
}
void LinearAngleSpringConstraint::setAngleLimit( const double lo, const double hi )
{
    setAngleLimit( osg::Vec2( lo, hi ) );
}
void LinearAngleSpringConstraint::setLinearStiffness( float stiffness )
{
    _data->_stiffness[ 0 ] = btScalar( stiffness );
    setSpringData( _data.get() );
}
void LinearAngleSpringConstraint::setAngleStiffness( float stiffness )
{
    _data->_stiffness[ 3 ] = btScalar( stiffness );
    setSpringData( _data.get() );
}
void LinearAngleSpringConstraint::setLinearDamping( float damping )
{
    _data->_damping[ 0 ] = btScalar( damping );
    setSpringData( _data.get() );
}
void LinearAngleSpringConstraint::setAngleDamping( float damping )
{
    _data->_damping[ 3 ] = btScalar( damping );
    setSpringData( _data.get() );
}

bool LinearAngleSpringConstraint::operator==( const LinearAngleSpringConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool LinearAngleSpringConstraint::operator!=( const LinearAngleSpringConstraint& rhs ) const
{
    return(
        ( _axis != rhs._axis ) ||
        ( *_data != *( rhs._data ) ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void LinearAngleSpringConstraint::createConstraint()
{
    if( _constraint != NULL )
    {
        delete _constraint;
        _constraint = NULL;
    }

    _constraint = LinearSpringConstraint::internalCreateSpringConstraint(
        this, _data.get(), _axis, _pivotPoint );

    setDirty( _constraint == NULL );
}




FixedConstraint::FixedConstraint()
  : Constraint()
{
}
FixedConstraint::FixedConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : Constraint( rbA, rbB )
{
    setDirty();
}
FixedConstraint::FixedConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform )
  : Constraint( rbA, rbAXform, rbB, rbBXform )
{
    setDirty();
}
FixedConstraint::FixedConstraint( const FixedConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop )
{
}
FixedConstraint::~FixedConstraint()
{
}

btGeneric6DofConstraint* FixedConstraint::getAsBtGeneric6Dof() const
{
    return( static_cast< btGeneric6DofConstraint* >( getConstraint() ) );
}

void FixedConstraint::createConstraint()
{
    if( _rbA == NULL )
    {
        osg::notify( osg::INFO ) << "createConstraint: _rbA == NULL." << std::endl;
        return;
    }

    if( _constraint != NULL )
    {
        delete _constraint;
        _constraint = NULL;
    }


    // A's reference frame is just the COM offset.
    osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( _rbA->getMotionState() );
    if( motion == NULL )
    {
        osg::notify( osg::WARN ) << "SliderConstraint: Invalid MotionState." << std::endl;
        return;
    }
    osg::Vec3 invCom = -( motion->getCenterOfMass() );
    osg::Vec3 scale = motion->getScale();
    osg::Vec3 scaledInvCom = osg::Vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
    const osg::Matrix invACOM( osg::Matrix::translate( scaledInvCom ) );
    btTransform rbAFrame = osgbCollision::asBtTransform( invACOM );


    btTransform rbBFrame; // OK to not initialize.
    if( _rbB != NULL )
    {
        // Create a matrix that puts A in B's coordinate space.
        //
        //   1. Inverse B center of mass offset.
        motion = dynamic_cast< osgbDynamics::MotionState* >( _rbB->getMotionState() );
        if( motion == NULL )
        {
            osg::notify( osg::WARN ) << "SliderConstraint: Invalid MotionState." << std::endl;
            return;
        }
        invCom = -( motion->getCenterOfMass() );
        scale = motion->getScale();
        scaledInvCom = osg::Vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
        const osg::Matrix invBCOM( osg::Matrix::translate( scaledInvCom ) );
        //
        //   3. Transform from B's origin to A's origin.
        const osg::Matrix rbBToRbA( osg::Matrix::inverse( orthonormalize( _rbBXform ) ) *
            orthonormalize( _rbAXform ) );
        //
        //   4. The final rbB frame matrix.
        rbBFrame = osgbCollision::asBtTransform(
            invBCOM * rbBToRbA );
    }


    btGeneric6DofConstraint* cons;
    if( _rbB != NULL )
        cons = new btGeneric6DofConstraint( *_rbA, *_rbB, rbAFrame, rbBFrame, false );
    else
        cons = new btGeneric6DofConstraint( *_rbA, rbAFrame, true );
    cons->setAngularLowerLimit( btVector3( 0., 0., 0. ) );
    cons->setAngularUpperLimit( btVector3( 0., 0., 0. ) );
    _constraint = cons;

    setDirty( false );
}




PlanarConstraint::PlanarConstraint()
  : Constraint()
{
}
PlanarConstraint::PlanarConstraint( btRigidBody* rbA, btRigidBody* rbB,
        const osg::Vec2& loLimit, const osg::Vec2& hiLimit, const osg::Matrix& orient )
  : Constraint( rbA, rbB ),
    _loLimit( loLimit ),
    _hiLimit( hiLimit ),
    _orient( orient )
{
    setDirty( true );
}
PlanarConstraint::PlanarConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        const osg::Vec2& loLimit, const osg::Vec2& hiLimit, const osg::Matrix& orient )
  : Constraint( rbA, rbAXform ),
    _loLimit( loLimit ),
    _hiLimit( hiLimit ),
    _orient( orient )
{
    setDirty( true );
}
PlanarConstraint::PlanarConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform,
        const osg::Vec2& loLimit, const osg::Vec2& hiLimit, const osg::Matrix& orient )
  : Constraint( rbA, rbAXform, rbB, rbBXform ),
    _loLimit( loLimit ),
    _hiLimit( hiLimit ),
    _orient( orient )
{
    setDirty( true );
}
PlanarConstraint::PlanarConstraint( const PlanarConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _loLimit( rhs._loLimit ),
    _hiLimit( rhs._hiLimit ),
    _orient( rhs._orient )
{
    setDirty( true );
}
PlanarConstraint::~PlanarConstraint()
{
}

btGeneric6DofConstraint* PlanarConstraint::getAsBtGeneric6Dof() const
{
    return( static_cast< btGeneric6DofConstraint* >( getConstraint() ) );
}

void PlanarConstraint::setLowLimit( const osg::Vec2& loLimit )
{
    _loLimit = loLimit;
    setDirty( true );
}
void PlanarConstraint::setLowLimit( const double x, const double y )
{
    setLowLimit( osg::Vec2( x, y ) );
}
void PlanarConstraint::setHighLimit( const osg::Vec2& hiLimit )
{
    _hiLimit = hiLimit;
    setDirty( true );
}
void PlanarConstraint::setHighLimit( const double x, const double y )
{
    setHighLimit( osg::Vec2( x, y ) );
}
void PlanarConstraint::setOrientation( const osg::Matrix& orient )
{
    _orient = orient;
    setDirty( true );
}

bool PlanarConstraint::operator==( const PlanarConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool PlanarConstraint::operator!=( const PlanarConstraint& rhs ) const
{
    return(
        ( _loLimit != rhs._loLimit ) ||
        ( _hiLimit != rhs._hiLimit ) ||
        ( _orient != rhs._orient ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void PlanarConstraint::createConstraint()
{
    if( _rbA == NULL )
    {
        osg::notify( osg::INFO ) << "createConstraint: _rbA == NULL." << std::endl;
        return;
    }

    if( _constraint != NULL )
    {
        delete _constraint;
        _constraint = NULL;
    }


    // Planar and Box share common code to compute the Bullet constraint
    // reference frames.
    btTransform rbAFrame, rbBFrame;
    BoxConstraint::internalPlanarBoxFrameComputation(
        rbAFrame, rbBFrame, this, _orient );


    btGeneric6DofConstraint* cons;
    if( _rbB != NULL )
        cons = new btGeneric6DofConstraint( *_rbA, *_rbB, rbAFrame, rbBFrame, false );
    else
        cons = new btGeneric6DofConstraint( *_rbA, rbAFrame, true );
    cons->setAngularLowerLimit( btVector3( 0., 0., 0. ) );
    cons->setAngularUpperLimit( btVector3( 0., 0., 0. ) );

    const osg::Vec3 loLimit( _loLimit[ 0 ], _loLimit[ 1 ], 0. );
    const osg::Vec3 hiLimit( _hiLimit[ 0 ], _hiLimit[ 1 ], 0. );
    cons->setLinearLowerLimit( osgbCollision::asBtVector3( loLimit ) );
    cons->setLinearUpperLimit( osgbCollision::asBtVector3( hiLimit ) );

    _constraint = cons;

    setDirty( false );
}




BoxConstraint::BoxConstraint()
  : Constraint()
{
}
BoxConstraint::BoxConstraint( btRigidBody* rbA, btRigidBody* rbB,
        const osg::Vec3& loLimit, const osg::Vec3& hiLimit, const osg::Matrix& orient )
  : Constraint( rbA, rbB ),
    _loLimit( loLimit ),
    _hiLimit( hiLimit ),
    _orient( orient )
{
    setDirty( true );
}
BoxConstraint::BoxConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        const osg::Vec3& loLimit, const osg::Vec3& hiLimit, const osg::Matrix& orient )
  : Constraint( rbA, rbAXform ),
    _loLimit( loLimit ),
    _hiLimit( hiLimit ),
    _orient( orient )
{
    setDirty( true );
}
BoxConstraint::BoxConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform,
        const osg::Vec3& loLimit, const osg::Vec3& hiLimit, const osg::Matrix& orient )
  : Constraint( rbA, rbAXform, rbB, rbBXform ),
    _loLimit( loLimit ),
    _hiLimit( hiLimit ),
    _orient( orient )
{
    setDirty( true );
}
BoxConstraint::BoxConstraint( const BoxConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _loLimit( rhs._loLimit ),
    _hiLimit( rhs._hiLimit ),
    _orient( rhs._orient )
{
    setDirty( true );
}
BoxConstraint::~BoxConstraint()
{
}

btGeneric6DofConstraint* BoxConstraint::getAsBtGeneric6Dof() const
{
    return( static_cast< btGeneric6DofConstraint* >( getConstraint() ) );
}

void BoxConstraint::setLowLimit( const osg::Vec3& loLimit )
{
    _loLimit = loLimit;
    setDirty( true );
}
void BoxConstraint::setLowLimit( const double x, const double y, const double z )
{
    setLowLimit( osg::Vec3( x, y, z ) );
}
void BoxConstraint::setHighLimit( const osg::Vec3& hiLimit )
{
    _hiLimit = hiLimit;
    setDirty( true );
}
void BoxConstraint::setHighLimit( const double x, const double y, const double z )
{
    setHighLimit( osg::Vec3( x, y, z ) );
}
void BoxConstraint::setOrientation( const osg::Matrix& orient )
{
    _orient = orient;
    setDirty( true );
}

bool BoxConstraint::operator==( const BoxConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool BoxConstraint::operator!=( const BoxConstraint& rhs ) const
{
    return(
        ( _loLimit != rhs._loLimit ) ||
        ( _hiLimit != rhs._hiLimit ) ||
        ( _orient != rhs._orient ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void BoxConstraint::createConstraint()
{
    if( _rbA == NULL )
    {
        osg::notify( osg::INFO ) << "createConstraint: _rbA == NULL." << std::endl;
        return;
    }

    if( _constraint != NULL )
    {
        delete _constraint;
        _constraint = NULL;
    }


    // Planar and Box share common code to compute the Bullet constraint
    // reference frames.
    btTransform rbAFrame, rbBFrame;
    BoxConstraint::internalPlanarBoxFrameComputation(
        rbAFrame, rbBFrame, this, _orient );


    btGeneric6DofConstraint* cons;
    if( _rbB != NULL )
        cons = new btGeneric6DofConstraint( *_rbA, *_rbB, rbAFrame, rbBFrame, false );
    else
        cons = new btGeneric6DofConstraint( *_rbA, rbAFrame, true );
    cons->setAngularLowerLimit( btVector3( 0., 0., 0. ) );
    cons->setAngularUpperLimit( btVector3( 0., 0., 0. ) );

    cons->setLinearLowerLimit( osgbCollision::asBtVector3( _loLimit ) );
    cons->setLinearUpperLimit( osgbCollision::asBtVector3( _hiLimit ) );

    _constraint = cons;

    setDirty( false );
}

void BoxConstraint::internalPlanarBoxFrameComputation(
        btTransform& aFrame, btTransform& bFrame,
        Constraint* cons, const osg::Matrix& orient )
{
    // Remove any translation that might be in the orient matrix.
    osg::Matrix orientation( orient );
    orientation.setTrans( 0., 0., 0. );

    btRigidBody* rbA, * rbB;
    cons->getRigidBodies( rbA, rbB );


    // Create a matrix that puts A in B's coordinate space, accounting
    // for orientation of the constraint axes.
    //
    //   1. Inverse A center of mass offset.
    osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( rbA->getMotionState() );
    if( motion == NULL )
    {
        osg::notify( osg::WARN ) << "InternalCreateSpring: Invalid MotionState." << std::endl;
        return;
    }
    osg::Vec3 invCom = -( motion->getCenterOfMass() );
    osg::Vec3 scale = motion->getScale();
    osg::Vec3 scaledInvCom = osg::Vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
    const osg::Matrix invACOM( osg::Matrix::translate( scaledInvCom ) );
    //
    //   2. Transform A back to the origin.
    const osg::Matrix invAXform( osg::Matrix::inverse( cons->getAXform() ) );
    //
    //   3. The final rbA frame matrix.
    aFrame = osgbCollision::asBtTransform( 
        orientation * invAXform * invACOM );


    if( rbB != NULL )
    {
        // Create a matrix that orients the spring axis/point in B's coordinate space.
        //
        //   1. Inverse B center of mass offset.
        motion = dynamic_cast< osgbDynamics::MotionState* >( rbB->getMotionState() );
        if( motion == NULL )
        {
            osg::notify( osg::WARN ) << "InternalCreateSpring: Invalid MotionState." << std::endl;
            return;
        }
        invCom = -( motion->getCenterOfMass() );
        scale = motion->getScale();
        scaledInvCom = osg::Vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
        const osg::Matrix invBCOM( osg::Matrix::translate( scaledInvCom ) );
        //
        //   2. Transform B back to the origin.
        const osg::Matrix invBXform( osg::Matrix::inverse( cons->getBXform() ) );
        //
        //   3. The final rbB frame matrix.
        bFrame = osgbCollision::asBtTransform( 
            orientation * invBXform * invBCOM );
    }
}




HingeConstraint::HingeConstraint()
  : Constraint(),
    _axis( 0., 0., 1. ),
    _pivotPoint( 0., 0., 0. ),
    _limit( osg::PI, osg::PI )
{
}
HingeConstraint::HingeConstraint( btRigidBody* rbA, btRigidBody* rbB,
        const osg::Vec3& axis, const osg::Vec3& pivotPoint,
        const osg::Vec2& limit )
  : Constraint( rbA, rbB ),
    _axis( axis ),
    _pivotPoint( pivotPoint ),
    _limit( limit )
{
}
HingeConstraint::HingeConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform,
        const osg::Vec3& axis, const osg::Vec3& pivotPoint,
        const osg::Vec2& limit )
  : Constraint( rbA, rbAXform, rbB, rbBXform ),
    _axis( axis ),
    _pivotPoint( pivotPoint ),
    _limit( limit )
{
}
HingeConstraint::HingeConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        const osg::Vec3& axis, const osg::Vec3& pivotPoint,
        const osg::Vec2& limit )
  : Constraint( rbA, rbAXform ),
    _axis( axis ),
    _pivotPoint( pivotPoint ),
    _limit( limit )
{
}
HingeConstraint::HingeConstraint( const HingeConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _axis( rhs._axis ),
    _pivotPoint( rhs._pivotPoint ),
    _limit( rhs._limit )
{
}
HingeConstraint::~HingeConstraint()
{
}

btHingeConstraint* HingeConstraint::getAsBtHinge() const
{
    return( static_cast< btHingeConstraint* >( getConstraint() ) );
}

void HingeConstraint::setAxis( const osg::Vec3& axis )
{
    _axis = axis;
    setDirty( true );
}
void HingeConstraint::setAxis( const double x, const double y, const double z )
{
    setAxis( osg::Vec3( x, y, z ) );
}
void HingeConstraint::setPivotPoint( const osg::Vec3& wcPoint )
{
    _pivotPoint = wcPoint;
    setDirty( true );
}
void HingeConstraint::setPivotPoint( const double x, const double y, const double z )
{
    setPivotPoint( osg::Vec3( x, y, z ) );
}
void HingeConstraint::setLimit( const osg::Vec2& limit )
{
    _limit = limit;

    if( !getDirty() && ( _constraint != NULL ) )
    {
        // Dynamically modify the existing constraint.
        btHingeConstraint* cons = getAsBtHinge();
        cons->setLimit( _limit[ 0 ], _limit[ 1 ] );
    }
    else
        setDirty();
}
void HingeConstraint::setLimit( const double lo, const double hi )
{
    setLimit( osg::Vec2( lo, hi ) );
}

bool HingeConstraint::operator==( const HingeConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool HingeConstraint::operator!=( const HingeConstraint& rhs ) const
{
    return(
        ( _axis != rhs._axis ) ||
        ( _pivotPoint != rhs._pivotPoint ) ||
        ( _limit != rhs._limit ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}


void HingeConstraint::createConstraint()
{
    if( _rbA == NULL )
    {
        osg::notify( osg::INFO ) << "createConstraint: _rbA == NULL." << std::endl;
        return;
    }

    if( _constraint != NULL )
    {
        delete _constraint;
        _constraint = NULL;
    }


    // Put pivot point and axis into A's space.
    // 
    // 1. Inverse A center of mass:
    osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( _rbA->getMotionState() );
    if( motion == NULL )
    {
        osg::notify( osg::WARN ) << "HingeConstraint: Invalid MotionState." << std::endl;
        return;
    }
    osg::Vec3 invCom = -( motion->getCenterOfMass() );
    osg::Vec3 scale = motion->getScale();
    osg::Vec3 scaledInvCom = osg::Vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
    osg::Matrix invACom = osg::Matrix::translate( scaledInvCom );
    //
    // 2. Inverse A transform.
    osg::Matrix invAXform = osg::Matrix::inverse( orthonormalize( _rbAXform ) );
    //
    // 3. A's orientation.
    osg::Matrix rbAOrient( orthonormalize( _rbAXform ) );
    rbAOrient.setTrans( 0., 0., 0. );
    //
    // Transform the point and axis.
    btVector3 pivotInA( osgbCollision::asBtVector3(
        _pivotPoint * invAXform * rbAOrient * invACom ) );
    btVector3 axisInA( osgbCollision::asBtVector3( _axis * rbAOrient ) );


    btVector3 pivotInB, axisInB;
    if( _rbB != NULL )
    {
        // Put pivot point and axis into B's space.
        // 
        // 1. Inverse B center of mass:
        osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( _rbB->getMotionState() );
        if( motion == NULL )
        {
            osg::notify( osg::WARN ) << "HingeConstraint: Invalid MotionState." << std::endl;
            return;
        }
        invCom = -( motion->getCenterOfMass() );
        scale = motion->getScale();
        scaledInvCom = osg::Vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
        osg::Matrix invBCom = osg::Matrix::translate( scaledInvCom );
        //
        // 2. Inverse A transform.
        osg::Matrix invBXform = osg::Matrix::inverse( orthonormalize( _rbBXform ) );
        //
        // 3. B's orientation.
        osg::Matrix rbBOrient( orthonormalize( _rbBXform ) );
        rbBOrient.setTrans( 0., 0., 0. );
        //
        // Transform the point and axis.
        pivotInB = osgbCollision::asBtVector3(
            _pivotPoint * invBXform * rbBOrient * invBCom );
        axisInB = osgbCollision::asBtVector3( _axis * rbBOrient );
    }


    btHingeConstraint* hinge;
    if( _rbB != NULL )
        hinge = new btHingeConstraint( *_rbA, *_rbB,
                pivotInA, pivotInB, axisInA, axisInB, false );
    else
        hinge = new btHingeConstraint( *_rbA, pivotInA, axisInA, false );

    hinge->setLimit( _limit[ 0 ], _limit[ 1 ] );

    _constraint = hinge;
}




CardanConstraint::CardanConstraint()
  : Constraint(),
    _axisA( 0., 1., 0. ),
    _axisB( 1., 0., 0. ),
    _point( 0., 0., 0. )
{
}
CardanConstraint::CardanConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : Constraint( rbA, rbB ),
    _axisA( 0., 1., 0. ),
    _axisB( 1., 0., 0. ),
    _point( 0., 0., 0. )
{
}
CardanConstraint::CardanConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform,
        const osg::Vec3& axisA, const osg::Vec3& axisB, const osg::Vec3& point )
  : Constraint( rbA, rbAXform, rbB, rbBXform ),
    _axisA( axisA ),
    _axisB( axisB ),
    _point( point )
{
}
CardanConstraint::CardanConstraint( const CardanConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _axisA( rhs._axisA ),
    _axisB( rhs._axisB ),
    _point( rhs._point )
{
}
CardanConstraint::~CardanConstraint()
{
}

btUniversalConstraint* CardanConstraint::getAsBtUniversal() const
{
    return( static_cast< btUniversalConstraint* >( getConstraint() ) );
}

void CardanConstraint::setAxisA( const osg::Vec3& axisA )
{
    _axisA = axisA;
    setDirty( true );
}
void CardanConstraint::setAxisA( const double x, const double y, const double z )
{
    setAxisA( osg::Vec3( x, y, z ) );
}
void CardanConstraint::setAxisB( const osg::Vec3& axisB )
{
    _axisB = axisB;
    setDirty( true );
}
void CardanConstraint::setAxisB( const double x, const double y, const double z )
{
    setAxisB( osg::Vec3( x, y, z ) );
}
void CardanConstraint::setAnchorPoint( const osg::Vec3& wcPoint )
{
    _point = wcPoint;
    setDirty( true );
}
void CardanConstraint::setAnchorPoint( const double x, const double y, const double z )
{
    setAnchorPoint( osg::Vec3( x, y, z ) );
}

bool CardanConstraint::operator==( const CardanConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool CardanConstraint::operator!=( const CardanConstraint& rhs ) const
{
    return(
        ( _axisA != rhs._axisA ) ||
        ( _axisB != rhs._axisB ) ||
        ( _point != rhs._point ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void CardanConstraint::createConstraint()
{
    if( ( _rbA == NULL ) || ( _rbB == NULL ) )
    {
        osg::notify( osg::INFO ) << "createConstraint: _rbA == NULL or _rbB == NULL." << std::endl;
        return;
    }

    if( _constraint != NULL )
    {
        delete _constraint;
        _constraint = NULL;
    }


    // Transform the world coordinate _axisA into A's local coordinates.
    osg::Matrix aOrient = orthonormalize( _rbAXform) ;
    aOrient.setTrans( 0., 0., 0. );
    btVector3 localAxisA = osgbCollision::asBtVector3(
        _axisA * osg::Matrix::inverse( aOrient ) );
    localAxisA.normalize();


    // Force _axisB to be orthogonal to _axisA.
    osg::Vec3 c = _axisA ^ _axisB;
    osg::Vec3 axisB( c ^ _axisA );

    // Transform the world coordinate _axisB into B's local coordinates.
    osg::Matrix bOrient = orthonormalize( _rbBXform );
    bOrient.setTrans( 0., 0., 0. );
    btVector3 localAxisB = osgbCollision::asBtVector3(
        axisB * osg::Matrix::inverse( bOrient ) );
    localAxisB.normalize();


    btVector3 btPt( osgbCollision::asBtVector3( _point ) );
    btUniversalConstraint* cons = new btUniversalConstraint( *_rbA, *_rbB,
        btPt, localAxisA, localAxisB );

    _constraint = cons;

    setDirty( false );
}
    



BallAndSocketConstraint::BallAndSocketConstraint()
  : Constraint()
{
}
BallAndSocketConstraint::BallAndSocketConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : Constraint( rbA, rbB )
{
}
BallAndSocketConstraint::BallAndSocketConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB, const osg::Matrix& rbBXform, const osg::Vec3& wcPoint )
  : Constraint( rbA, rbAXform, rbB, rbBXform ),
    _point( wcPoint )
{
}
BallAndSocketConstraint::BallAndSocketConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            const osg::Vec3& wcPoint )
  : Constraint( rbA, rbAXform ),
    _point( wcPoint )
{
}
BallAndSocketConstraint::BallAndSocketConstraint( const BallAndSocketConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _point( rhs._point )
{
}
BallAndSocketConstraint::~BallAndSocketConstraint()
{
}

btPoint2PointConstraint* BallAndSocketConstraint::getAsBtPoint2Point() const
{
    return( static_cast< btPoint2PointConstraint* >( getConstraint() ) );
}

void BallAndSocketConstraint::setPoint( const osg::Vec3& wcPoint )
{
    _point = wcPoint;
    setDirty();
}
void BallAndSocketConstraint::setPoint( const double x, const double y, const double z )
{
    setPoint( osg::Vec3( x, y, z ) );
}

bool BallAndSocketConstraint::operator==( const BallAndSocketConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool BallAndSocketConstraint::operator!=( const BallAndSocketConstraint& rhs ) const
{
    return(
        ( _point != rhs._point ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void BallAndSocketConstraint::createConstraint()
{
    if( _rbA == NULL )
    {
        osg::notify( osg::INFO ) << "createConstraint: _rbA == NULL." << std::endl;
        return;
    }

    if( _constraint != NULL )
    {
        delete _constraint;
        _constraint = NULL;
    }


    // Create a matrix that returns A to the origin.
    //
    //   1. Inverse A center of mass offset.
    osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( _rbA->getMotionState() );
    if( motion == NULL )
    {
        osg::notify( osg::WARN ) << "InternalCreateSpring: Invalid MotionState." << std::endl;
        return;
    }
    osg::Vec3 invCom = -( motion->getCenterOfMass() );
    osg::Vec3 scale = motion->getScale();
    osg::Vec3 scaledInvCom = osg::Vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
    const osg::Matrix invACOM( osg::Matrix::translate( scaledInvCom ) );
    //
    //   2. Transform A back to the origin.
    const osg::Matrix invAXform( osg::Matrix::inverse( orthonormalize( _rbAXform ) ) );
    //
    //   3. The final rbA frame matrix.
    const osg::Matrix aXform( invAXform * invACOM );

    // And now compute the WC point in rbA space:
    const btVector3 aPt = osgbCollision::asBtVector3( _point * aXform );


    btVector3 bPt;
    if( _rbB != NULL )
    {
        // Create a matrix that returns B to the origin.
        //
        //   1. Inverse B center of mass offset.
        motion = dynamic_cast< osgbDynamics::MotionState* >( _rbB->getMotionState() );
        if( motion == NULL )
        {
            osg::notify( osg::WARN ) << "InternalCreateSpring: Invalid MotionState." << std::endl;
            return;
        }
        invCom = -( motion->getCenterOfMass() );
        scale = motion->getScale();
        scaledInvCom = osg::Vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
        const osg::Matrix invBCOM( osg::Matrix::translate( scaledInvCom ) );
        //
        //   2. Transform B back to the origin.
        const osg::Matrix invBXform( osg::Matrix::inverse( orthonormalize( _rbBXform ) ) );
        //
        //   3. The final rbB frame matrix.
        const osg::Matrix bXform = invBXform * invBCOM;

        // And now compute the WC point in rbB space:
        bPt = osgbCollision::asBtVector3( _point * bXform );
    }


    btPoint2PointConstraint* cons;
    if( _rbB != NULL )
        cons = new btPoint2PointConstraint( *_rbA, *_rbB, aPt, bPt );
    else
        cons = new btPoint2PointConstraint( *_rbA, aPt );
    _constraint = cons;

    setDirty( false );
}




RagdollConstraint::RagdollConstraint()
  : Constraint(),
    _axis( 1., 0., 0. ),
    _angle( osg::PI_2 )
{
}
RagdollConstraint::RagdollConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : Constraint( rbA, rbB ),
    _axis( 1., 0., 0. ),
    _angle( osg::PI_2 )
{
}
RagdollConstraint::RagdollConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        const osg::Vec3& wcPoint, const osg::Vec3& wcAxis, const double angleRadians )
  : Constraint( rbA, rbAXform ),
    _point( wcPoint ),
    _axis( wcAxis ),
    _angle( angleRadians )
{
}
RagdollConstraint::RagdollConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
        btRigidBody* rbB, const osg::Matrix& rbBXform,
        const osg::Vec3& wcPoint, const osg::Vec3& wcAxis, const double angleRadians )
  : Constraint( rbA, rbAXform, rbB, rbBXform ),
    _point( wcPoint ),
    _axis( wcAxis ),
    _angle( angleRadians )
{
}
RagdollConstraint::RagdollConstraint( const RagdollConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _point( rhs._point ),
    _axis( rhs._axis ),
    _angle( rhs._angle )
{
}
RagdollConstraint::~RagdollConstraint()
{
}

btConeTwistConstraint* RagdollConstraint::getAsBtConeTwist() const
{
    return( static_cast< btConeTwistConstraint* >( getConstraint() ) );
}

void RagdollConstraint::setPoint( const osg::Vec3& wcPoint )
{
    _point = wcPoint;
    setDirty( true );
}
void RagdollConstraint::setPoint( const double x, const double y, const double z )
{
    setPoint( osg::Vec3( x, y, z ) );
}
void RagdollConstraint::setAxis( const osg::Vec3& wcAxis )
{
    _axis = wcAxis;
    setDirty( true );
}
void RagdollConstraint::setAxis( const double x, const double y, const double z )
{
    setAxis( osg::Vec3( x, y, z ) );
}
void RagdollConstraint::setAngle( const double angleRadians )
{
    _angle = angleRadians;

    if( !getDirty() && ( _constraint != NULL ) )
    {
        // Dynamically modify the existing constraint.
        btConeTwistConstraint* cons = getAsBtConeTwist();
        cons->setLimit( 4, _angle );
        cons->setLimit( 5, _angle );
    }
    else
        setDirty();
}

bool RagdollConstraint::operator==( const RagdollConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool RagdollConstraint::operator!=( const RagdollConstraint& rhs ) const
{
    return(
        ( _point != rhs._point ) ||
        ( _axis != rhs._axis ) ||
        ( _angle != rhs._angle ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void RagdollConstraint::createConstraint()
{
    if( _rbA == NULL )
    {
        osg::notify( osg::INFO ) << "createConstraint: _rbA == NULL." << std::endl;
        return;
    }

    if( _constraint != NULL )
    {
        delete _constraint;
        _constraint = NULL;
    }


    // Orientation matrix for the ragdoll x-axis / point.
    osg::Vec3 localAxis( _axis );
    localAxis.normalize();
    const osg::Matrix orientation =
        osg::Matrix::rotate( osg::Vec3( 1., 0., 0. ), localAxis ) *
        osg::Matrix::translate( _point );


    // Create a matrix that puts A in B's coordinate space, accounting
    // for orientation of the constraint axes.
    //
    //   1. Inverse A center of mass offset.
    osgbDynamics::MotionState* motion = dynamic_cast< osgbDynamics::MotionState* >( _rbA->getMotionState() );
    if( motion == NULL )
    {
        osg::notify( osg::WARN ) << "InternalCreateSpring: Invalid MotionState." << std::endl;
        return;
    }
    osg::Vec3 invCom = -( motion->getCenterOfMass() );
    osg::Vec3 scale = motion->getScale();
    osg::Vec3 scaledInvCom = osg::Vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
    const osg::Matrix invACOM( osg::Matrix::translate( scaledInvCom ) );
    //
    //   2. Transform A back to the origin.
    const osg::Matrix invAXform( osg::Matrix::inverse( orthonormalize( _rbAXform ) ) );
    //
    //   3. The final rbA frame matrix.
    btTransform rbAFrame = osgbCollision::asBtTransform( 
        orientation * invAXform * invACOM );


    btTransform rbBFrame;
    if( _rbB != NULL )
    {
        // Create a matrix that orients the spring axis/point in B's coordinate space.
        //
        //   1. Inverse B center of mass offset.
        motion = dynamic_cast< osgbDynamics::MotionState* >( _rbB->getMotionState() );
        if( motion == NULL )
        {
            osg::notify( osg::WARN ) << "InternalCreateSpring: Invalid MotionState." << std::endl;
            return;
        }
        invCom = -( motion->getCenterOfMass() );
        scale = motion->getScale();
        scaledInvCom = osg::Vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
        const osg::Matrix invBCOM( osg::Matrix::translate( scaledInvCom ) );
        //
        //   2. Transform B back to the origin.
        const osg::Matrix invBXform( osg::Matrix::inverse( orthonormalize( _rbBXform ) ) );
        //
        //   3. The final rbB frame matrix.
        rbBFrame = osgbCollision::asBtTransform( 
            orientation * invBXform * invBCOM );
    }


    btConeTwistConstraint* cons;
    if( _rbB != NULL )
        cons = new btConeTwistConstraint( *_rbA, *_rbB, rbAFrame, rbBFrame );
    else
        cons = new btConeTwistConstraint( *_rbA, rbAFrame );

    // The btConeTwistConstraint cone is along axis x (index 3).
    // It allows an assymetrical cone spread in y and z (indices 4 and 5).
    // We set both y and z to the same angle for a symmetrical spread.
    cons->setLimit( 4, _angle );
    cons->setLimit( 5, _angle );

    _constraint = cons;

    setDirty( false );
}




WheelSuspensionConstraint::WheelSuspensionConstraint()
  : Constraint(),
    _springAxis( 0., 0., 1. ),
    _axleAxis( 0., 1., 0. ),
    _linearLimit( -1., 1. ),
    _angleLimit( -osg::PI_4, osg::PI_4 ),
    _point( 0., 0., 0. )
{
}
WheelSuspensionConstraint::WheelSuspensionConstraint( btRigidBody* rbA, btRigidBody* rbB,
        const osg::Vec3& springAxis, const osg::Vec3& axleAxis,
        const osg::Vec2& linearLimit, const osg::Vec2& angleLimit,
        const osg::Vec3& point )
  : Constraint( rbA, rbB ),
    _springAxis( springAxis ),
    _axleAxis( axleAxis ),
    _linearLimit( linearLimit ),
    _angleLimit( angleLimit ),
    _point( point )
{
}
WheelSuspensionConstraint::WheelSuspensionConstraint( const WheelSuspensionConstraint& rhs, const osg::CopyOp& copyop )
  : Constraint( rhs, copyop ),
    _springAxis( rhs._springAxis ),
    _axleAxis( rhs._axleAxis ),
    _linearLimit( rhs._linearLimit ),
    _angleLimit( rhs._angleLimit ),
    _point( rhs._point )
{
}
WheelSuspensionConstraint::~WheelSuspensionConstraint()
{
}

btHinge2Constraint* WheelSuspensionConstraint::getAsBtHinge2() const
{
    return( static_cast< btHinge2Constraint* >( getConstraint() ) );
}

void WheelSuspensionConstraint::setSpringAxis( const osg::Vec3& springAxis )
{
    _springAxis = springAxis;
    setDirty( true );
}
void WheelSuspensionConstraint::setSpringAxis( const double x, const double y, const double z )
{
    setSpringAxis( osg::Vec3( x, y, z ) );
}
void WheelSuspensionConstraint::setAxleAxis( const osg::Vec3& axleAxis )
{
    _axleAxis = axleAxis;
    setDirty( true );
}
void WheelSuspensionConstraint::setAxleAxis( const double x, const double y, const double z )
{
    setAxleAxis( osg::Vec3( x, y, z ) );
}
void WheelSuspensionConstraint::setLinearLimit( const osg::Vec2& linearLimit )
{
    _linearLimit = linearLimit;

    if( !getDirty() && ( _constraint != NULL ) )
    {
        // Dynamically modify the existing constraint.
        btHinge2Constraint* cons = getAsBtHinge2();
        cons->setLinearLowerLimit( btVector3( 0., 0., _linearLimit[ 0 ] ) );
        cons->setLinearUpperLimit( btVector3( 0., 0., _linearLimit[ 1 ] ) );
    }
    else
        setDirty();
}
void WheelSuspensionConstraint::setLinearLimit( const double lo, const double hi )
{
    setLinearLimit( osg::Vec2( lo, hi ) );
}
void WheelSuspensionConstraint::setAngleLimit( const osg::Vec2& limitRadians )
{
    _angleLimit = limitRadians;

    if( !getDirty() && ( _constraint != NULL ) )
    {
        // Dynamically modify the existing constraint.
        btHinge2Constraint* cons = getAsBtHinge2();
        cons->setLowerLimit( _angleLimit[ 0 ] );
        cons->setUpperLimit( _angleLimit[ 1 ] );
    }
    else
        setDirty();
}
void WheelSuspensionConstraint::setAngleLimit( const double lo, const double hi )
{
    setAngleLimit( osg::Vec2( lo, hi ) );
}
void WheelSuspensionConstraint::setAnchorPoint( const osg::Vec3& wcPoint )
{
    _point = wcPoint;
    setDirty( true );
}
void WheelSuspensionConstraint::setAnchorPoint( const double x, const double y, const double z )
{
    setAnchorPoint( osg::Vec3( x, y, z ) );
}

bool WheelSuspensionConstraint::operator==( const WheelSuspensionConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool WheelSuspensionConstraint::operator!=( const WheelSuspensionConstraint& rhs ) const
{
    return(
        ( _springAxis != rhs._springAxis ) ||
        ( _axleAxis != rhs._axleAxis ) ||
        ( _linearLimit != rhs._linearLimit ) ||
        ( _angleLimit != rhs._angleLimit ) ||
        ( _point != rhs._point ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void WheelSuspensionConstraint::createConstraint()
{
    if( ( _rbA == NULL ) || ( _rbB == NULL ) )
    {
        osg::notify( osg::INFO ) << "createConstraint: _rbA == NULL or _rbB == NULL." << std::endl;
        return;
    }

    if( _constraint != NULL )
    {
        delete _constraint;
        _constraint = NULL;
    }


    // Force _axleAxis to be orthogonal to _springAxis.
    osg::Vec3 c = _springAxis ^ _axleAxis;
    btVector3 axle = osgbCollision::asBtVector3( c ^ _springAxis );

    btVector3 spring = osgbCollision::asBtVector3( _springAxis );
    btVector3 anchor = osgbCollision::asBtVector3( _point );

    // Everything is in world coords, just create the constraint.
    btHinge2Constraint* cons = new btHinge2Constraint( *_rbA, *_rbB, anchor, spring, axle );

    cons->setLinearLowerLimit( btVector3( 0., 0., _linearLimit[ 0 ] ) );
    cons->setLinearUpperLimit( btVector3( 0., 0., _linearLimit[ 1 ] ) );
    cons->setLowerLimit( _angleLimit[ 0 ] );
    cons->setUpperLimit( _angleLimit[ 1 ] );
    cons->setEquilibriumPoint();

    _constraint = cons;

    setDirty( false );
}



// osgbDynamics
}
