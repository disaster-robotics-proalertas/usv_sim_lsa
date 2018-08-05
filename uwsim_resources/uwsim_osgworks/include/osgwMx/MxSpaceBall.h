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

#ifndef __OSGWMX_MX_SPACE_BALL_H__
#define __OSGWMX_MX_SPACE_BALL_H__ 1


#include <osgwMx/Export.h>
#include <osgwMx/MxCore.h>
#include <osgwMx/MxUtils.h>
#include <osg/Object>
#include <osg/ref_ptr>
#include <osg/Vec2f>

namespace osgwMx {


/** \class MxSpaceBall MxSpaceBall.h <osgwMx/MxSpaceBall.h>
\brief A base class to support various SpaceBall SDKs.

To support a new SDK, such as VR Juggler, derive a class from MxSpaceBall.
Implement a function in your class to receive events (either handle events
sent from the SDK, or a function to poll the SDK device once per frame).
Call that function anything you want -- it's your class's specific function,
and the MxSpaceBall base class imposes no policy on its interface or how you
implement it.

In that event handling function, call into this base class as appropriate
in response to events or device state. Functions are provided for a variety
of axes (VRPN calls these 'Analogs', and an unsigned int button mask
(VRPN 'Buttons').

This base class "has a" FunctionalMap that maps buttons to functions. See
setButtons() and setFunctionalMap(). This base class creates a default
FunctionalMap with a default button-to-function mapping. Derived classes don't
need to re-configure this mapping, but can if desired. The intention of this
design is that the owning application will load a config file for the functional
map that is specific to a particular game pad device and/or VR device SDK (such
as DirectX or VR Juggler):

\code
MxSpaceBall::setFunctionalMap( static_cast< FunctionalMap >(
        osgDB::readObjectFile( "my-spaceball-config.osg" ) ) );
\endcode

An example FunctionalMap config file is in data/default-spaceball.osg.

This base class "has a" instance of an MxCore object. Functions to set stick
positions and button state result in immediate calls into MxCore. The
MxCore object is fully exposed. An application can retrieve the MxCore
managed matrix as desired. Your MxSpaceBall- derived class can also call
directly into the \c _mxCore member variable, if necessary, to support
functionality unavailable in the MxSpaceBall base class.
*/
class OSGWMX_EXPORT MxSpaceBall : public osg::Object
{
public:
    MxSpaceBall();
    MxSpaceBall( const MxSpaceBall& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgwMx,MxSpaceBall);

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
	
	/** Set the axis values [-1.0,1.0]

    Different axes (SpaceBall has 6) control movement and rotation.
	setAxes(float,float,float,float,float,float) modifies the MxCore
	state based on the specified values once per function call.

    Movement is scaled by the MoveModifyScaleSpeedDown and MoveModifyScaleSpeedUp buttons
	(defined in the FunctionalMap). MoveModifyScaleSpeedDown decreases speed
    to 0.33x, and MoveModifyScaleSpeedUp increases speed to 3x. Movement is further scaled
    by MxCore::setMoveScale(). While movement scaling is sufficient for small demos, it's
    often insufficient to move at a specified rate.
	Consider using setAxes(float,float,float,float,float,float,double) instead.
	See also setAxisRate() and setAxisRates().
    */
    virtual bool setAxes( const float x, const float y, const float z, const float h, const float p, const float b );
    void getAxes( float& x, float& y, float& z, float& h, float& p, float& b ) const
    {
        x = _movementAnalogs[0];
		y = _movementAnalogs[1];
		z = _movementAnalogs[2];
		// 0,1,2 are p,h,b order
		p = _rotationAnalogs[0];
		h = _rotationAnalogs[1];
		b = _rotationAnalogs[2];
    }

	/** Set the axis values with a specified rate.

    Behavior is identical to that of setAxes(float,float,float,float,float,float),
	except that values are taken as normalized percentages in the range -1.0 to 1.0 of a desired
    movement/rotation rate in units/second. The rate for the axis is set with
    setAxisRate() (see also setAxisRates()).
    
    \param deltaSeconds Specifies the elapsed time in seconds since the last call to
    setAxes(float,float,float,float,float,float,double). This function computes the
	delta motion/rotation based on \c _translateRate, \c _rotateRate and \c deltaSeconds,
	then scales that by the input values. */
    virtual bool setAxes( const float x, const float y, const float z, const float h, const float p, const float b, const double deltaSeconds );

    /** Set the axis dead zone.

    Specify a non-zero dead zone to eliminate non-zero values from
    idle analog controls.

    setAxis() checks its input parameters
    against the specified dead zone value. If the absolute value of
    a parameter is less that \c dz, the value is zeroed. The default
    dead zone value is 0.0.

    Most analog produce non-zero values for controls that are
    actually idle. Call setAxisDeadZone() to eliminate this issue.

    As an example usage, where setAxis is called with \c val values in the
	range -1 to 1, a \c dz dead zone value of 0.05 would mean that 5% around
	the analog control center position would be treated as a zero value. */
    void setAxisDeadZone( const float dz ) { _deadZone = dz; }
    float getAxisDeadZone() const { return( _deadZone ); }

    /** Set the translate (movement) rate in units/second.
	See setAxes(float,float,float,float,float,float,double).
    The default is 1.0.
    */
    void setTranslateRate( const double translateRate )
    {
        _translateRate = translateRate;
    }

	/** Set both the movement and rotation rates. 
	setAxes(float,float,float,float,float,float,double).
    The default for \c translateRate movement is 1.0 units/second, and the default
    for \c rotateRate rotation is 60.0 degrees/second. */
    void setAllRates( const double translateRate, const double rotateRate )
    {
        _translateRate = translateRate; _rotateRate = rotateRate;
    }
    void getAllRates( double& translateRate, double& rotateRate )
    {
        translateRate = _translateRate; rotateRate = _rotateRate;
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


    /** Set the current button state.

    Each button's function is determined by the functional map.
    See setFunctionalMap().

    Note that derived classes should call setButtons even when all buttons are zero.

    \param buttons A bit mask composed of MxSpaceBall::Buttons.
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
    handlers, such as MxEventHandler and MxSpaceBall. */
    void setMxCore( osgwMx::MxCore* mxCore ) { _mxCore = mxCore; }
    /** Get the MxCore for access to the managed matrix. */
    MxCore* getMxCore() { return( _mxCore.get() ); }
    const MxCore* getMxCore() const { return( _mxCore.get() ); }
    
protected:
    virtual ~MxSpaceBall();

    inline float deadZone( const float value )
    {
        return( ( osg::absolute< float >( value ) > _deadZone ) ? value : 0.f );
    }

    virtual void internalTranslate( const float x, const float y, const float z );
    virtual void internalRotate( const float x, const float y, const float z );

    osg::Vec3f _movementAnalogs;
    osg::Vec3f _rotationAnalogs;
    unsigned int _buttons;

    float _deadZone;

    double _translateRate, _rotateRate;

    osg::ref_ptr< MxCore > _mxCore;

    osg::ref_ptr< FunctionalMap > _map;
    FunctionalMap::FunctionType _moveMode;
    FunctionalMap::FunctionType _rotateMode;
};


// osgwMx
}


// __OSGWMX_MX_SPACE_BALL_H__
#endif
