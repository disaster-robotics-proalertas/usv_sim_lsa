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

#ifndef __OSGWMX_MX_GAME_PAD_H__
#define __OSGWMX_MX_GAME_PAD_H__ 1


#include <osgwMx/Export.h>
#include <osgwMx/MxCore.h>
#include <osgwMx/MxUtils.h>
#include <osg/Object>
#include <osg/ref_ptr>
#include <osg/Vec2f>

namespace osgwMx {


/** \class MxGamePad MxGamePad.h <osgwMx/MxGamePad.h>
\brief A base class to support various GamePad SDKs.

To support a new SDK, such as VR Juggler, derive a class from MxGamePad.
Implement a function in your class to receive events (either handle events
sent from the SDK, or a function to poll the SDK device once per frame).
Call that function anything you want -- it's your class's specific function,
and the MxGamePad base class imposes no policy on its interface or how you
implement it.

In that event handling function, call into this base class as appropriate
in response to events or device state. Functions are provided for left stick
x and y axes, right stick x and y axes, and an unsigned int button mask.
The "DPad" (above the left stick on many game pads) is not currently supported.

This base class "has a" FunctionalMap that maps buttons to functions. See
setButtons() and setFunctionalMap(). This base class creates a default
FunctionalMap with a default button-to-function mapping. Derived classes don't
need to re-configure this mapping, but can if desired. The intention of this
design is that the owning application will load a config file for the functional
map that is specific to a particular game pad device and/or VR device SDK (such
as DirectX or VR Juggler):

\code
MxGamePad::setFunctionalMap( static_cast< FunctionalMap >(
        osgDB::readObjectFile( "my-gamepad-config.osg" ) ) );
\endcode

An example FunctionalMap config file is in data/default-gamepad.osg.

This base class "has a" instance of an MxCore object. Functions to set stick
positions and button state result in immediate calls into MxCore. The
MxCore object is fully exposed. An application can retrieve the MxCore
managed matrix as desired. Your MxGamePad- derived class can also call
directly into the \c _mxCore member variable, if necessary, to support
functionality unavailable in the MxGamePad base class.
*/
class OSGWMX_EXPORT MxGamePad : public osg::Object
{
public:
    MxGamePad();
    MxGamePad( const MxGamePad& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgwMx,MxGamePad);

    /** Set the left stick position.

    Left stick controls movement. setLeftStick(float,float) modifies the MxCore position based on the
    specified values once per function call. For example, calling setLeftStick( -1., 0. )
    twice in a row will move the MxCore position -2.0 in x.

    Left stick \c x axis moves the position left/right. With no buttons depressed, \c y
    axis moves the position forward/backward. If the BottomButton is depressed ("RGB"
    buttons in front of right stick), then left stick \c y axis moves the position up
    and down. Negative values move left, backward, and down, while positive values move
    right, forward, and up.

    Movement is scaled by the right shoulder buttons. RightShoulderBottom decreases speed
    to 0.33x, and RightShoulderTop increases speed to 3x. Movement is further scaled
    by MxCore::setMoveScale(). While movement scaling is sufficient for small demos, it's
    often insufficient to move at a specified rate. Consider using setLeftStick(float,float,double)
    instead. See also setStickRate() and setStickRates().
    */
    virtual bool setLeftStick( const float x, const float y );
    void getLeftStick( float& x, float& y ) const
    {
        x = _leftStick[0]; y = _leftStick[1];
    }
    /** Move at a specified rate.

    Behavior is identical to that of setLeftStick(float,float), except that \c x and \c y
    values are taken as normalized percentages in the range -1.0 to 1.0 of a desired
    movement rate in units/second. The movement rate for the left stick is set with
    setStickRate() (see also setStickRates()).
    
    \param deltaSeconds Specifies the elapsed time in seconds since the last call to
    setLeftStick(float,float,double). This function computes the delta motion based on
    \c _leftRate and \c deltaSeconds, then scales that motion by \c x and \c y. */
    virtual bool setLeftStick( const float x, const float y, const double deltaSeconds );

    /** Set the right stick position.

    Right stick controls rotation. The x value rotates around the current
    view up vector. The y value rotates around the "right" vector (the
    cross product of the view direction and view up vector).

    Positive values rotate counterclockwise. For example, if the MxCore
    class is being used to manage a view matrix, and setRightStick is called
    with a positive x value, this rotates the view counterclockwise around
    the up vector, resulting in "turning your head left". In the same situation,
    a positive y value "looks up".
    */
    virtual bool setRightStick( const float x, const float y );
    void getRightStick( float& x, float& y ) const
    {
        x = _rightStick[0]; y = _rightStick[1];
    }
    /** Rotate at a specified rate.

    Behavior is identical to that of setRightStick(float,float), except that \c x and \c y
    values are taken as normalized percentages in the range -1.0 to 1.0 of a desired
    rotation rate in degrees/second. The rotation rate for the right stick is set with
    setStickRates().
    
    \param deltaSeconds Specifies the elapsed time in seconds since the last call to
    setRightStick(float,float,double). This function computes the delta rotation based on
    \c _rightRate and \c deltaSeconds, then scales that rotation by \c x and \c y. */
    virtual bool setRightStick( const float x, const float y, const double deltaSeconds );

    /** Set the stick dead zone.

    Specify a non-zero dead zone to eliminate non-zero values from
    idle game pad sticks.

    Both setLeftStick() and setRightStick() check their input parameters
    against the specified dead zone value. If the absolute value of
    a parameter is less that \c dz, the value is zeroed. The default
    dead zone value is 0.0.

    Most game pad sticks produce non-zero values for sticks that are
    actually idle. Call setStickDeadZone() to eliminate this issue.

    As an example usage, where setLeftStick and setRightStick are called
    with \c x and \c y values in the range -1 to 1, a \c dz dead zone value
    of 0.05 would mean that 5% around the stick center position would be
    treated as a zero value. */
    void setStickDeadZone( const float dz ) { _deadZone = dz; }
    float getStickDeadZone() const { return( _deadZone ); }

    /** Set the left stick (movement) rate in units/second. See setLeftStick(float,float,double).
    The default is 1.0.
    */
    void setStickRate( const double leftRate )
    {
        _leftRate = leftRate;
    }
    /** Set both the left and right stick (movement and rotation) rates. See
    setLeftStick(float,float,double) and setRightStick(float,float,double).
    The default for \c leftRate movement is 1.0 units/second, and the default
    for \c rightRate rotation is 60.0 degrees/second. */
    void setStickRates( const double leftRate, const double rightRate )
    {
        _leftRate = leftRate; _rightRate = rightRate;
    }
    void getStickRates( double& leftRate, double& rightRate )
    {
        leftRate = _leftRate; rightRate = _rightRate;
    }

    /** Set the movement mode to one of the valid move modes defined
    in the FunctionalMap::FunctionType enum.

    FunctionType declares several enums, not all of which are move types.
    If you call this function with an enum that isn't a valid move type,
    the function displays a warning and returns.

    Default is FunctionalMap::MoveModeLocal. */
    void setMoveMode( const FunctionalMap::FunctionType mode );
    FunctionalMap::FunctionType getMoveMode() const
    {
        return( _moveMode );
    }
    void cycleMoveMode();

    /** Set the rotation mode to one of the valid rotate modes defined
    in the FunctionalMap::FunctionType enum.

    FunctionType declares several enums, not all of which are rotate types.
    If you call this function with an enum that isn't a valid rotate type,
    the function displays a warning and returns.

    Default is FunctionalMap::RotateModeLocal. */
    void setRotateMode( const FunctionalMap::FunctionType mode );
    FunctionalMap::FunctionType getRotateMode() const
    {
        return( _rotateMode );
    }
    void cycleRotateMode();

    /** Enumerant button values. Bitwise OR enumerants that correspond to
    pressed device buttons, and pass the result to setButtons(). */
    typedef enum {
        Button0  = ( 0x1 <<  0 ),
        Button1  = ( 0x1 <<  1 ),
        Button2  = ( 0x1 <<  2 ),
        Button3  = ( 0x1 <<  3 ),
        Button4  = ( 0x1 <<  4 ),
        Button5  = ( 0x1 <<  5 ),
        Button6  = ( 0x1 <<  6 ),
        Button7  = ( 0x1 <<  7 ),
        Button8  = ( 0x1 <<  8 ),
        Button9  = ( 0x1 <<  9 ),
        Button10 = ( 0x1 << 10 ),
        Button11 = ( 0x1 << 11 ),
        Button12 = ( 0x1 << 12 ),
        Button13 = ( 0x1 << 13 ),
        Button14 = ( 0x1 << 14 ),
        Button15 = ( 0x1 << 15 ),
        Button16 = ( 0x1 << 16 ),
        Button17 = ( 0x1 << 17 ),
        Button18 = ( 0x1 << 18 ),
        Button19 = ( 0x1 << 19 ),
        Button20 = ( 0x1 << 20 ),
        Button21 = ( 0x1 << 21 ),
        Button22 = ( 0x1 << 22 ),
        Button23 = ( 0x1 << 23 ),
        Button24 = ( 0x1 << 24 ),
        Button25 = ( 0x1 << 25 ),
        Button26 = ( 0x1 << 26 ),
        Button27 = ( 0x1 << 27 ),
        Button28 = ( 0x1 << 28 ),
        Button29 = ( 0x1 << 29 ),
        Button30 = ( 0x1 << 30 ),
        Button31 = ( 0x1 << 31 )
    } Buttons;
    /** Set the current button state.

    Each button's function is determined by the functional map.
    See setFunctionalMap().

    Note that derived classes should call setButtons even when all buttons are zero.

    \param buttons A bit mask composed of MxGamePad::Buttons.
    */
    virtual void setButtons( const unsigned int buttons );
    unsigned int getButtons() const { return( _buttons ); }

    /** \brief Set the button state, passing elapsed time in seconds.

    Use this function when a button is mapped to a function that requires the time
    delta, such as FunctionalMap::MoveUpAtRate or FunctionalMap::MoveDownAtRate.
    */
    virtual void setButtons( const unsigned int buttons, const double deltaSeconds );

    /** Access the mapping of buttons to functionality. */
    void setFunctionalMap( osgwMx::FunctionalMap* map ) { _map = map; }
    osgwMx::FunctionalMap* getFunctionalMap() { return( _map.get() ); }
    const osgwMx::FunctionalMap* getFunctionalMap() const { return( _map.get() ); }

    /** Set the MxCore. This allows sharing an MxCore between multiple device
    handlers, such as MxEventHandler and MxGamePad. */
    void setMxCore( osgwMx::MxCore* mxCore ) { _mxCore = mxCore; }
    /** Get the MxCore for access to the managed matrix. */
    MxCore* getMxCore() { return( _mxCore.get() ); }
    const MxCore* getMxCore() const { return( _mxCore.get() ); }

    /** Set the point of rotation for the right stick rotations. This allows the
    user to press button 10 and rotate about an arbitrary point in space with 
    the right stick control.*/
    void setRotationPoint( const osg::Vec3d& point )
    {
        _mxCore->setOrbitCenterPoint( point );
    }
    
protected:
    virtual ~MxGamePad();

    inline float deadZone( const float value )
    {
        return( ( osg::absolute< float >( value ) > _deadZone ) ? value : 0.f );
    }

    virtual void internalLeftStick( const float x, const float y );
    virtual void internalRightStick( const float x, const float y );

    osg::Vec2f _leftStick;
    osg::Vec2f _rightStick;
    unsigned int _buttons;

    float _deadZone;

    double _leftRate, _rightRate;

    osg::ref_ptr< MxCore > _mxCore;

    osg::ref_ptr< FunctionalMap > _map;
    FunctionalMap::FunctionType _moveMode;
    FunctionalMap::FunctionType _rotateMode;
};


// osgwMx
}


// __OSGWMX_MX_GAME_PAD_H__
#endif
