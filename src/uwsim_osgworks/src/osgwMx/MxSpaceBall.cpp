/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgWorks is (C) Copyright 2009-2012 by Kenneth Mark Bryden
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


#include <osgwMx/MxSpaceBall.h>
#include <osg/io_utils>
#include <osg/Notify>

#include <osg/Math>


namespace osgwMx {


MxSpaceBall::MxSpaceBall()
  : osg::Object(),
    _movementAnalogs( osg::Vec3f( 0.f, 0.f, 0.f  ) ),
    _rotationAnalogs( osg::Vec3f( 0.f, 0.f, 0.f  ) ),
    _buttons( 0 ),
    _deadZone( 0.f ),
    _translateRate( 1. ),
    _rotateRate( 60. ),
    _moveMode( FunctionalMap::MoveModeLocal ),
    _rotateMode( FunctionalMap::RotateModeLocal )
{
    _mxCore = new osgwMx::MxCore;

    // Create a default functional map.
    _map = new osgwMx::FunctionalMap;
	// <<<>>> These could be rearranged for SpaceBalls which have 1-9 and A,B,C
    _map->configure( Button0, FunctionalMap::JumpToWorldOrigin );
    _map->configure( Button1, FunctionalMap::LevelView );
    _map->configure( Button2, FunctionalMap::MoveModifyUpDown );
    _map->configure( Button3, FunctionalMap::JumpToHomePosition );
    _map->configure( Button6, FunctionalMap::MoveModifyScaleSpeedDown );
    _map->configure( Button7, FunctionalMap::MoveModifyScaleSpeedUp );
    _map->configure( Button8, FunctionalMap::MoveModeWorld );
    _map->configure( Button9, FunctionalMap::MoveModeConstrained );
    _map->configure( Button10, FunctionalMap::RotateModeOrbit );
}
MxSpaceBall::MxSpaceBall( const MxSpaceBall& rhs, const osg::CopyOp& copyop )
  : osg::Object( rhs, copyop ),
    _movementAnalogs( rhs._movementAnalogs ),
    _rotationAnalogs( rhs._rotationAnalogs ),
    _buttons( rhs._buttons ),
    _deadZone( rhs._deadZone ),
    _translateRate( rhs._translateRate ),
    _rotateRate( rhs._rotateRate ),
    _mxCore( new osgwMx::MxCore( *( rhs._mxCore ), copyop ) ),
    _map( new osgwMx::FunctionalMap( *( rhs._map ), copyop ) ),
    _moveMode( rhs._moveMode ),
    _rotateMode( rhs._rotateMode )
{
}
MxSpaceBall::~MxSpaceBall()
{
}

bool MxSpaceBall::setAxes( const float x, const float y, const float z, const float h, const float p, const float b )
{
    _movementAnalogs.set( x, y, z );
	_rotationAnalogs.set( p, h, b ); // order is p,h,b

    // Zero the values if they fall within the dead zone.
	const float myX( deadZone( x ) );
	const float myY( deadZone( y ) );
	const float myZ( deadZone( z ) );
	const float myP( deadZone( p ) );
	const float myH( deadZone( h ) );
	const float myB( deadZone( b ) );

	if( myX == 0 && myY == 0 && myZ == 0  && myP == 0  && myH == 0  && myB == 0 )
	{
		return( false );
	}
    
    internalTranslate( myX, myY, myZ );
	internalRotate( myP, myB, myH ); //P=x, B=y, H=z order
    
    return( true );
}
bool MxSpaceBall::setAxes( const float x, const float y, const float z, const float h, const float p, const float b, const double deltaSeconds )
{
	// <<<>>> this stuff should be refactored and shared with the above
	_movementAnalogs.set( x, y, z );
	_rotationAnalogs.set( p, h, b ); // order is p,h,b

	// Zero the values if they fall within the dead zone.
	const float myX( deadZone( x ) );
	const float myY( deadZone( y ) );
	const float myZ( deadZone( z ) );
	const float myP( deadZone( p ) );
	const float myH( deadZone( h ) );
	const float myB( deadZone( b ) );

	if( myX == 0 && myY == 0 && myZ == 0  && myP == 0  && myH == 0  && myB == 0 )
	{
		return( false );
	}
    
    // How far do we go at 100% movement?
    const float maxDistance = (float)( _translateRate * deltaSeconds );

	// How far do we turn at 100% rotation?
	const float maxDegrees = (float)( _rotateRate * deltaSeconds );

	internalTranslate( myX * maxDistance, myY * maxDistance, myZ * maxDistance);
	internalRotate( myP * maxDegrees, myB * maxDegrees, myH * maxDegrees); //P=x, B=y, H=z order
    
    return( true );
}
void MxSpaceBall::internalTranslate( const float x, const float y, const float z )
{
    osg::Vec3d movement;
	// SpaceBall moves on all three axes all the time and doesn't support MoveModifyUpDown
    movement.set( x, -y, z );

    switch( getMoveMode() )
    {
    default:
        osg::notify( osg::WARN ) << "Unsupported move mode: \"" << FunctionalMap::asString( getMoveMode() ) << "\"" << std::endl;
        // Intentional fallthrough.
    case FunctionalMap::MoveModeLiteral:
        _mxCore->moveLiteral( movement );
        break;
    case FunctionalMap::MoveModeLocal:
        _mxCore->moveLocal( movement );
        break;
    case FunctionalMap::MoveModeConstrained:
        _mxCore->moveConstrained( movement );
        break;
    case FunctionalMap::MoveModeOriented:
        _mxCore->moveOriented( movement );
        break;
    case FunctionalMap::MoveModeWorld:
        _mxCore->moveWorld( movement );
        break;
    }
}


void MxSpaceBall::internalRotate( const float x, const float y, const float z )
{
    // Input is degrees, but MxCore wants radians.
    const double myX = osg::DegreesToRadians( x );
    const double myY = osg::DegreesToRadians( y );
	const double myZ = osg::DegreesToRadians( z );

    // spaceball has a dedicated roll axis and doesn't need RotateModifyRoll
    _mxCore->rotateLocal( myY, _mxCore->getDir() ); // bank/roll is on "Y" into screen axis
    switch( getRotateMode() )
    {
    default:
        osg::notify( osg::WARN ) << "Unsupported rotate mode: \"" << FunctionalMap::asString( getRotateMode() ) << "\"" << std::endl;
        // Intentional fallthrough.
    case FunctionalMap::RotateModeLocal:
		// myY is handled commonly, above
        _mxCore->rotateLocal( myZ, _mxCore->getUp() );
        _mxCore->rotateLocal( myX, _mxCore->getCross() );
        break;
    case FunctionalMap::RotateModeOrbit:
        _mxCore->rotateOrbit( myZ, _mxCore->getUp());
        _mxCore->rotateOrbit( myX, _mxCore->getCross());
        break;
    case FunctionalMap::RotateModeArcball:
        osg::notify( osg::WARN ) << "RotateModeArcball not yet implemented." << std::endl;
        break;
    }
}

void MxSpaceBall::setMoveMode( const FunctionalMap::FunctionType mode )
{
    if( FunctionalMap::validMoveMode( mode ) )
        _moveMode = mode;
    else
        osg::notify( osg::WARN ) << "Invalid move mode: \"" <<
            FunctionalMap::asString( mode ) << "\"" << std::endl;
}
void MxSpaceBall::cycleMoveMode()
{
    setMoveMode( FunctionalMap::cycleMoveMode( getMoveMode() ) );
}
void MxSpaceBall::setRotateMode( const FunctionalMap::FunctionType mode )
{
    if( FunctionalMap::validRotateMode( mode ) )
        _rotateMode = mode;
    else
        osg::notify( osg::WARN ) << "Invalid rotate mode: \"" <<
            FunctionalMap::asString( mode ) << "\"" << std::endl;
}
void MxSpaceBall::cycleRotateMode()
{
    setRotateMode( FunctionalMap::cycleRotateMode( getRotateMode() ) );
}


void MxSpaceBall::setButtons( const unsigned int buttons )
{
    // Determine which buttons just entered a pressed or released state.
    const unsigned int deltaPressed = ( buttons ^ _buttons ) & buttons;
    const unsigned int deltaReleased = ( buttons ^ _buttons ) & _buttons;


    //
    // Handle buttons that have just been pressed.
    _map->setFromBitmask( deltaPressed );

    if( _map->isSet( FunctionalMap::JumpToHomePosition ) )
        _mxCore->reset();
    if( _map->isSet( FunctionalMap::JumpToWorldOrigin ) )
        _mxCore->setPosition( osg::Vec3( 0., 0., 0. ) );
    if( _map->isSet( FunctionalMap::LevelView ) )
        _mxCore->level();

    if( _map->isSet( FunctionalMap::CycleMoveMode ) )
        cycleMoveMode();
    if( _map->isSet( FunctionalMap::MoveModeLocal ) )
        setMoveMode( FunctionalMap::MoveModeLocal );
    if( _map->isSet( FunctionalMap::MoveModeConstrained ) )
        setMoveMode( FunctionalMap::MoveModeConstrained );
    if( _map->isSet( FunctionalMap::MoveModeOriented ) )
        setMoveMode( FunctionalMap::MoveModeOriented );
    if( _map->isSet( FunctionalMap::MoveModeWorld ) )
        setMoveMode( FunctionalMap::MoveModeWorld );

    if( _map->isSet( FunctionalMap::CycleRotateMode ) )
        cycleRotateMode();
    if( _map->isSet( FunctionalMap::RotateModeLocal ) )
        setMoveMode( FunctionalMap::RotateModeLocal );
    if( _map->isSet( FunctionalMap::RotateModeOrbit ) )
        setMoveMode( FunctionalMap::RotateModeOrbit );
    if( _map->isSet( FunctionalMap::RotateModeArcball ) )
        setMoveMode( FunctionalMap::RotateModeArcball );

    // Scale movement based on MoveModifyScaleSpeedDown/MoveModifyScaleSpeedUp button state.
    if( _map->isSet( FunctionalMap::MoveModifyScaleSpeedDown ) )
        _mxCore->setMoveScale( osg::Vec3d( .33, .33, .33 ) );
    else if( _map->isSet( FunctionalMap::MoveModifyScaleSpeedUp ) )
        _mxCore->setMoveScale( osg::Vec3d( 3., 3., 3. ) );


    // Handle buttons that have just been released.
    _map->setFromBitmask( deltaReleased );

    // If either the ScaleUp or ScaleDown buttons were released, restore
    // the move speed.
    if( _map->isSet( FunctionalMap::MoveModifyScaleSpeedDown ) ||
        _map->isSet( FunctionalMap::MoveModifyScaleSpeedUp ) )
        _mxCore->setMoveScale( osg::Vec3d( 1., 1., 1. ) );


    _buttons = buttons;
    _map->setFromBitmask( buttons );
}

void MxSpaceBall::setButtons( const unsigned int buttons, const double deltaSeconds )
{
    setButtons( buttons );

    // How far do we go at 100% movement?
    const float maxDistance = (float)( _translateRate * deltaSeconds );

    osg::Vec3 movement;
    if( _map->isSet( FunctionalMap::MoveUpAtRate ) )
        movement[1] = -maxDistance;
    else if( _map->isSet( FunctionalMap::MoveDownAtRate ) )
        movement[1] = maxDistance;

    if( _map->isSet( FunctionalMap::MoveModeWorld ) )
    {
        _mxCore->moveWorld( movement );
        return;
    }
    if( _map->isSet( FunctionalMap::MoveModeConstrained ) )
    {
        _mxCore->moveConstrained( movement );
        return;
    }
    if( _map->isSet( FunctionalMap::MoveModeOriented ) )
    {
        _mxCore->moveOriented( movement );
        return;
    }

    ///By default we will move in local coordinate space
    _mxCore->moveLocal( movement );
}


// osgwMx
}
