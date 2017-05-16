/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgWorks is (C) Copyright 2009-2011 by Kenneth Mark Bryden
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


// this code is inspired by, but not actually derived from the osgVRPN code in
// Mike Weiblen's osgToy http://mew.cx/osg/

#include <osgwMx/MxSpaceBallVRPN.h>
#include <osg/Notify>

namespace osgwMx {


inline float normalizeAxisValue( float value )
{
	// empirical testing with a 3dconnexion 4000FLX on VRPN showed no axis values > .40
	return( value  / 0.4 );
}


MxSpaceBallVRPN::MxSpaceBallVRPN(const std::string &deviceName)
  : MxSpaceBall(), _deviceName(deviceName), _vrpnButton(NULL), _vrpnAnalog(NULL), _buttons( 0 ), _deltaSeconds( 0.0 )
{
    // Base class stick units are normalized -1 to 1. Dead zone of 0.05
    // means that stick movement in range -0.05 to 0.05 will be ignored.
    setAxisDeadZone( 0.05f );
	
	// setup default device name if not provided
	if(_deviceName.empty())
		_deviceName = "Spaceball0@localhost";

    openVRPNDevice(_deviceName.c_str());
}
MxSpaceBallVRPN::MxSpaceBallVRPN( const MxSpaceBallVRPN& rhs, const osg::CopyOp& copyop )
  : MxSpaceBall( rhs, copyop ), _deviceName(rhs._deviceName), _vrpnButton(rhs._vrpnButton), _vrpnAnalog(rhs._vrpnAnalog), _buttons( 0 ), _deltaSeconds( 0.0 )
{
	// Due to the callbacks, an already-setup MxSpaceBallVRPN probably isn't copyable, but we'll try.
	openVRPNDevice(_deviceName.c_str());
}

MxSpaceBallVRPN::~MxSpaceBallVRPN()
{
    freeVRPNDevice();
}

bool MxSpaceBallVRPN::poll( const double deltaSeconds )
{
	_deltaSeconds = deltaSeconds; // to pass through the callback
	if( _vrpnAnalog == 0 && _vrpnButton == 0 )
		return( false );

	// run the mainloop to fire the callbacks
	if(_vrpnAnalog) _vrpnAnalog->mainloop();
	if(_vrpnButton) _vrpnButton->mainloop();
	return( true ); // Success.
}

void MxSpaceBallVRPN::processButtons( const vrpn_BUTTONCB b )
{
	// 12 buttons (1-9, A-C) are defined here, add more to taste
	// we only decode as many as are provided by the button callback
	Buttons buttonMap[] = {
		Button0, Button1, Button2, Button3, Button4, Button5, Button6, Button7, Button8, Button9, Button10, Button11 // A,B,C are in here too
	};

    // Map VRPN SpaceBall buttons to base class button enums.
	if(b.button < (sizeof(buttonMap) / sizeof(Buttons))) // adjusts for more elements in array
	{
		if(b.state)
		{
			_buttons |= buttonMap[b.button];
		} // if
		else
		{
			_buttons &= ~(buttonMap[b.button]);
		} // else
	} // if

    // Must call into base class even if all buttons are zero
    // so that base class can detect deltas (press events).
    setButtons( _buttons, _deltaSeconds );
}

void MxSpaceBallVRPN::processAxes( const vrpn_ANALOGCB a )
{	
	// processAxes only gets called if we have at least 6 channels
	float x, y, z, h, p(a.channel[4]), b(a.channel[5]);

    // Axes 0-2: Move.
    // These are units to move in world coordinates per event or per frame.
    x = normalizeAxisValue(a.channel[0]);
    y = -normalizeAxisValue(a.channel[1]);
	z = normalizeAxisValue(a.channel[2]);

    // 3,4,5/h,p,b: Rotate.
    // Range +-1.0
	// <<<>>> check the order of these
    p = normalizeAxisValue(a.channel[3]);
    h = normalizeAxisValue(a.channel[4]);
	b = -normalizeAxisValue(a.channel[5]);

	setAxes(x, y, z, h, p, b, _deltaSeconds);
}



bool MxSpaceBallVRPN::openVRPNDevice(const char *deviceName)
{
	// register callbacks
	if(_vrpnAnalog = new vrpn_Analog_Remote(_deviceName.c_str()))
	{
		_vrpnAnalog->register_change_handler( this, &handle_analog);
	} // if
	else
	{
		osg::notify( osg::WARN ) << "MxSpaceBallVRPN: Failed to open analog remote." << std::endl;
	} // else

	// button callback
	if(_vrpnButton = new vrpn_Button_Remote(_deviceName.c_str()))
	{
		_vrpnButton->register_change_handler( this, &handle_button );
	} // if
	else
	{
		osg::notify( osg::WARN ) << "MxSpaceBallVRPN: Failed to open button remote." << std::endl;
	}

    return( _vrpnAnalog && _vrpnButton );
}


void MxSpaceBallVRPN::freeVRPNDevice()
{
	if(_vrpnButton) _vrpnButton->unregister_change_handler( this, &handle_button );
	if(_vrpnAnalog) _vrpnAnalog->unregister_change_handler( this, &handle_analog );
	delete _vrpnAnalog;
	delete _vrpnButton;
}

// this is static so it can be called from outside
void VRPN_CALLBACK MxSpaceBallVRPN::handle_analog( void* userData, const vrpn_ANALOGCB a )
{

	if(a.num_channel >= 6)
		static_cast<MxSpaceBallVRPN *>(userData)->processAxes(a); // turn userdata into "this"

#ifdef DEBUG
	int nbChannels = a.num_channel;
	for( int i=0; i < a.num_channel; i++ )
	{
		cout << a.channel[i] << " ";
	}
	cout << endl;
#endif
}

// this is static so it can be called from outside
void VRPN_CALLBACK MxSpaceBallVRPN::handle_button( void* userData, const vrpn_BUTTONCB b )
{
#ifdef DEBUG
	cout << "Button '" << b.button << "': " << b.state << endl;
#endif
	// turn userdata into "this" so we can call object methods out of a static method
	static_cast<MxSpaceBallVRPN *>(userData)->processButtons(b);
}


// osgwMx
}
