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


#include <osgwMx/MxGamePad.h>
#include <osg/io_utils>
#include <osg/Notify>

#include <osg/Math>


namespace osgwMx {


MxGamePad::MxGamePad()
  : osg::Object(),
    _leftStick( osg::Vec2f( 0.f, 0.f ) ),
    _rightStick( osg::Vec2f( 0.f, 0.f ) ),
    _buttons( 0 ),
    _deadZone( 0.f ),
    _leftRate( 1. ),
    _rightRate( 60. ),
    _moveMode( FunctionalMap::MoveModeLocal ),
    _rotateMode( FunctionalMap::RotateModeLocal )
{
    _mxCore = new osgwMx::MxCore;

    // Create a default functional map.
    _map = new osgwMx::FunctionalMap;
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
MxGamePad::MxGamePad( const MxGamePad& rhs, const osg::CopyOp& copyop )
  : osg::Object( rhs, copyop ),
    _leftStick( rhs._leftStick ),
    _rightStick( rhs._rightStick ),
    _buttons( rhs._buttons ),
    _deadZone( rhs._deadZone ),
    _leftRate( rhs._leftRate ),
    _rightRate( rhs._rightRate ),
    _mxCore( new osgwMx::MxCore( *( rhs._mxCore ), copyop ) ),
    _map( new osgwMx::FunctionalMap( *( rhs._map ), copyop ) ),
    _moveMode( rhs._moveMode ),
    _rotateMode( rhs._rotateMode )
{
}
MxGamePad::~MxGamePad()
{
}

bool MxGamePad::setLeftStick( const float x, const float y )
{
    _leftStick.set( x, y );

    // Zero the values if they fall within the dead zone.
    float myX( deadZone( x ) );
    float myY( deadZone( y ) );

    if( myX == 0 && myY == 0 )
    {
        return( false );
    }
    
    internalLeftStick( myX, myY );
    
    return( true );
}
bool MxGamePad::setLeftStick( const float x, const float y, const double deltaSeconds )
{
    _leftStick.set( x, y );

    // Zero the values if they fall within the dead zone.
    const float myX( deadZone( x ) );
    const float myY( deadZone( y ) );

    if( myX == 0 && myY == 0 )
    {
        return( false );
    }
    
    // How far do we go at 100% movement?
    const float maxDistance = (float)( _leftRate * deltaSeconds );

    internalLeftStick( myX * maxDistance, myY * maxDistance );
    
    return( true );
}
void MxGamePad::internalLeftStick( const float x, const float y )
{
    // Check for forward/backward or up/down.
    osg::Vec3d movement;
    if( _map->isSet( FunctionalMap::MoveModifyUpDown ) )
    {
        // Move left/right and up/down.
        // Positive values move up, so negate y.
        movement.set( x, -y, 0. );
    }
    else
    {
        // Move left/right and forwards/backwards.
        movement.set( x, 0., y );
    }

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
    case FunctionalMap::MoveModeOrbit:
        _mxCore->moveOrbit( y );
        break;
    }
}

bool MxGamePad::setRightStick( const float x, const float y )
{
    _rightStick.set( x, y );

    // Zero the values if they fall within the dead zone.
    float myX( deadZone( x ) );
    float myY( deadZone( y ) );

    if( myX == 0 && myY == 0 )
    {
        return( false );
    }
    
    internalRightStick( myX, myY );
    
    return( true );
}
bool MxGamePad::setRightStick( const float x, const float y, const double deltaSeconds )
{
    _rightStick.set( x, y );

    // Zero the values if they fall within the dead zone.
    float myX( deadZone( x ) );
    float myY( deadZone( y ) );

    if( myX == 0 && myY == 0 )
    {
        return( false );
    }

    // How far do we turn at 100% rotation?
    const float maxDegrees = (float)( _rightRate * deltaSeconds );
    
    internalRightStick( myX * maxDegrees, myY * maxDegrees );
    
    return( true );
}
void MxGamePad::internalRightStick( const float x, const float y )
{
    // Input is degrees, but MxCore wants radians.
    const double myX = osg::DegreesToRadians( x );
    const double myY = osg::DegreesToRadians( y );

    if( _map->isSet( FunctionalMap::RotateModifyRoll ) )
    {
        _mxCore->rotateLocal( myX, _mxCore->getDir() );
    }
    else
    {
        switch( getRotateMode() )
        {
        default:
            osg::notify( osg::WARN ) << "Unsupported rotate mode: \"" << FunctionalMap::asString( getRotateMode() ) << "\"" << std::endl;
            // Intentional fallthrough.
        case FunctionalMap::RotateModeLocal:
            _mxCore->rotateLocal( myX, _mxCore->getUp() );
            _mxCore->rotateLocal( myY, _mxCore->getCross() );
            break;
        case FunctionalMap::RotateModeOrbit:
            _mxCore->rotateOrbit( myX, _mxCore->getUp() );
            _mxCore->rotateOrbit( myY, _mxCore->getCross() );
            break;
        case FunctionalMap::RotateModeArcball:
            osg::notify( osg::WARN ) << "RotateModeArcball not yet implemented." << std::endl;
            break;
        }
    }
}

void MxGamePad::setMoveMode( const FunctionalMap::FunctionType mode )
{
    if( FunctionalMap::validMoveMode( mode ) )
        _moveMode = mode;
    else
        osg::notify( osg::WARN ) << "Invalid move mode: \"" <<
            FunctionalMap::asString( mode ) << "\"" << std::endl;
}
void MxGamePad::cycleMoveMode()
{
    setMoveMode( FunctionalMap::cycleMoveMode( getMoveMode() ) );
}
void MxGamePad::setRotateMode( const FunctionalMap::FunctionType mode )
{
    if( FunctionalMap::validRotateMode( mode ) )
        _rotateMode = mode;
    else
        osg::notify( osg::WARN ) << "Invalid rotate mode: \"" <<
            FunctionalMap::asString( mode ) << "\"" << std::endl;
}
void MxGamePad::cycleRotateMode()
{
    setRotateMode( FunctionalMap::cycleRotateMode( getRotateMode() ) );
}


void MxGamePad::setButtons( const unsigned int buttons )
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
    if( _map->isSet( FunctionalMap::MoveModeOrbit ) )
        setMoveMode( FunctionalMap::MoveModeOrbit );

    if( _map->isSet( FunctionalMap::CycleRotateMode ) )
        cycleRotateMode();
    if( _map->isSet( FunctionalMap::RotateModeLocal ) )
        setRotateMode( FunctionalMap::RotateModeLocal );
    if( _map->isSet( FunctionalMap::RotateModeOrbit ) )
        setRotateMode( FunctionalMap::RotateModeOrbit );
    if( _map->isSet( FunctionalMap::RotateModeArcball ) )
        setRotateMode( FunctionalMap::RotateModeArcball );

    // Scale movement based on right shoulder button state.
    if( _map->isSet( FunctionalMap::MoveModifyScaleSpeedDown ) )
        _mxCore->setMoveScale( osg::Vec3d( .33, .33, .33 ) );
    else if( _map->isSet( FunctionalMap::MoveModifyScaleSpeedUp ) )
        _mxCore->setMoveScale( osg::Vec3d( 3., 3., 3. ) );


    //
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

void MxGamePad::setButtons( const unsigned int buttons, const double deltaSeconds )
{
    setButtons( buttons );

    // How far do we go at 100% movement?
    const float maxDistance = (float)( _leftRate * deltaSeconds );

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
    
    // By default we will move in local coordinate space
    _mxCore->moveLocal( movement );
}


// osgwMx
}
