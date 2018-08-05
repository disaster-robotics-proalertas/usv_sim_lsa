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


#include <osgwMx/MxGamePadDX.h>
#include <osg/notify>

#include <windows.h>
#include <dinput.h>
#include <tchar.h>


namespace osgwMx {


// DX GamePad axis min/max are customizable. Our code sets them
// large, so that we maintain prevision during normalization to
// the range -1 to 1 (required by the base class MxGamePad).
#define MIN_AXIS_VALUE -10000.f
#define MAX_AXIS_VALUE 10000.f
#define HALF_AXIS_RANGE 10000.f

    
// Statics
bool MxGamePadDX::s_winClassRegistered( false );

static LRESULT CALLBACK WinProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam );



// Forward
BOOL CALLBACK EnumAxisCallback(const DIDEVICEOBJECTINSTANCE* pDIDOI, void* pContext );
inline float normalizeAxisValue( LONG value );



MxGamePadDX::MxGamePadDX()
  : MxGamePad(),
    _hDIWindow( 0 ),
    _pDI( 0 ),
    _pDIDevice( 0 ),
    _devCaps( DIDEVCAPS() )
{
    // Base class stick units are normalized -1 to 1. Dead zone of 0.05
    // means that stick movement in range -0.05 to 0.05 will be ignored.
    setStickDeadZone( 0.05f );

    // DirectInput requires a Window from the app, so try to create one now.
    // This window will be a top-level window (not a child window), but will be
    // invisible to the user.
    if( ( _hDIWindow = createInvisiWindow() ) == 0 )
        return;

    // try to open DirectInput 8 for use. This works as far back as Windows 95 (DirectX 8.0a sdk).
    if( openDirectInput() )
        selectFirstDevice();    // try to select the first gaming device on the system.
}
MxGamePadDX::MxGamePadDX( const MxGamePadDX& rhs, const osg::CopyOp& copyop )
  : MxGamePad( rhs, copyop ),
    _hDIWindow( rhs._hDIWindow ),
    _pDI( rhs._pDI ),
    _pDIDevice( rhs._pDIDevice ),
    _devCaps( rhs._devCaps )
{
}
MxGamePadDX::~MxGamePadDX()
{
    freeDirectInput();
    destroyWindow();
}

bool MxGamePadDX::poll( const double deltaSeconds )
{
    if( _pDIDevice == 0 )
        return( false );

    HRESULT rc = _pDIDevice->Poll();
    if( ( rc != DI_OK ) && ( rc != DI_NOEFFECT ) )
    {
        osg::notify( osg::WARN ) << "MxGamePadDX::poll: failure." << std::endl;
        _pDIDevice->Acquire();            // try to reacquire device if lost it
        return( false );
    }

    // read the raw device state information in DirectInput format.
    DIJOYSTATE2 devState;
    if( _pDIDevice->GetDeviceState( sizeof( DIJOYSTATE2 ), &devState ) != DI_OK )
    {
        osg::notify( osg::WARN ) << "MxGamePadDX::poll: GetDeviceState failed." << std::endl;
        return( false ); // Something went wrong.
    }

    // Button pressed or not pressed could alter behavior of sticks/dpad,
    // so process buttons first.
    processButtons( devState, deltaSeconds );
    processSticks( devState, deltaSeconds );
    processDPad( devState );

    return( true ); // Success.
}

void MxGamePadDX::processButtons( const DIJOYSTATE2& devState, const double deltaSeconds )
{
    unsigned int buttons( 0 );

    // Map DX GamePad buttons to base class button enums.
    if( devState.rgbButtons[ 0 ] )
        buttons |= Button2;
    if( devState.rgbButtons[ 1 ] )
        buttons |= Button1;
    if( devState.rgbButtons[ 2 ] )
        buttons |= Button3;
    if( devState.rgbButtons[ 3 ] )
        buttons |= Button0;
    if( devState.rgbButtons[ 4 ] )
        buttons |= Button5;
    if( devState.rgbButtons[ 5 ] )
        buttons |= Button4;
    if( devState.rgbButtons[ 6 ] )
        buttons |= Button7;
    if( devState.rgbButtons[ 7 ] )
        buttons |= Button6;

    // Must call into base class even if all buttons are zero
    // so that base class can detect deltas (press events).
    setButtons( buttons, deltaSeconds );
}

void MxGamePadDX::processSticks( const DIJOYSTATE2& devState, const double deltaSeconds )
{
    float x, y;

    // Left stick: Move.
    // Normalize values to range -1.0 to 1.0.
    // These are units to move in world coordinates per event or per frame.
    x = normalizeAxisValue( devState.lX );
    y = normalizeAxisValue( devState.lY );
    setLeftStick( x, y, deltaSeconds );

    // Right stick: Rotate.
    // Base class angle values are in degrees. By calling
    // normalizeAxisValue, we pass in -1 to 1 degrees.
    // Compensate for rotation as well:
    //  x value around up vector, positive values counterclockwise
    //  y value around right/cross vector, positive values counterclockwise
    //    NOTE .lZ is positive when pulled back. This is the opposite of
    //    the left gamepad stick.
    x = -normalizeAxisValue( devState.lRz );
    y = normalizeAxisValue( devState.lZ );
    setRightStick( x, y, deltaSeconds );
}

void MxGamePadDX::processDPad( const DIJOYSTATE2& devState )
{
}



HWND MxGamePadDX::createInvisiWindow()
{
    HINSTANCE hMod = ( HINSTANCE )( GetModuleHandle( 0 ) );
    LPCTSTR className = _T( "MxInputAdapterGamePadDirectInput Window" );
    if( !s_winClassRegistered )
    {
        s_winClassRegistered = true;

        WNDCLASSEX wc;
        memset(&wc, 0, sizeof(wc));
        wc.cbSize = sizeof(WNDCLASSEX);
        wc.lpfnWndProc = &WinProc;
        wc.hInstance = hMod;
        wc.lpszClassName = className;
        if (RegisterClassEx(&wc) == 0)
            return 0;
    }
    return( ::CreateWindow( className, _T( "DirectInput Window" ), WS_POPUP, 0, 0, 1, 1, 0, 0, hMod, 0 ) );
}

bool MxGamePadDX::openDirectInput()
{
    if( _pDI != 0 )
        return true;

    HRESULT rc = DirectInput8Create( GetModuleHandle( 0 ),
        DIRECTINPUT_VERSION, IID_IDirectInput8, ( VOID ** )&_pDI, 0 );
    if( rc != DI_OK )
        osg::notify( osg::WARN ) << "MxGamePadDX: Failed to open Direct Input." << std::endl;

    return( rc == DI_OK );
}

bool MxGamePadDX::selectFirstDevice()
{
    // if no devices attached to the system, then return failure.
    enumDevices();
    if( _devList.empty() )
        return( false );

    return( selectDevice( *( _devList.begin() ) ) );
}

bool MxGamePadDX::selectDevice( const DIDEVICEINSTANCE& device )
{
    // release any currently held device.
    freeDevice();

    // create a device interface for the specified device.
    if( _pDI->CreateDevice( device.guidInstance, &_pDIDevice, 0 ) != DI_OK )
    {
        osg::notify( osg::WARN ) << "MxGamePadDX: Unable to select device." << std::endl;
        _pDIDevice = 0;
        return( false );
    }

    // set the type of data format we want from DirectInput.
    if( _pDIDevice->SetDataFormat( &c_dfDIJoystick2 ) != DI_OK )
    {
        osg::notify( osg::WARN ) << "MxGamePadDX: SetDataFormat() failed." << std::endl;
        freeDevice();
        return( false );
    }

    // use the device in a non-exclusive manner. No need to acquire it exclusively.
    // background access does not require that the window be the currently active window,
    // and will not automatically unacquire the device when the window is not the active window.
    if( _pDIDevice->SetCooperativeLevel( _hDIWindow, DISCL_BACKGROUND | DISCL_NONEXCLUSIVE ) != DI_OK )
    {
        osg::notify( osg::WARN ) << "MxGamePadDX: SetCooperativeLevel() failed." << std::endl;
        freeDevice();
        return( false );
    }

    _devCaps.dwSize = sizeof( DIDEVCAPS );
    if( _pDIDevice->GetCapabilities( &_devCaps ) != DI_OK )
    {
        osg::notify( osg::WARN ) << "MxGamePadDX: GetCapabilities() failed." << std::endl;
        freeDevice();
        return( false );
    }

    // set range of axis
    if( _pDIDevice->EnumObjects( EnumAxisCallback, _pDIDevice, DIDFT_AXIS ) != DI_OK )
    {
        osg::notify( osg::WARN ) << "MxGamePadDX: EnumObjects() failed." << std::endl;
        freeDevice();
        return( false );
    }

    HRESULT rc;
    // try to acquire the specified device for use.
    if( ( ( rc = _pDIDevice->Acquire() ) != DI_OK ) &&
        ( rc != S_FALSE ) )
    {
        osg::notify( osg::WARN ) << "MxGamePadDX: Failure to acquire device." << std::endl;
        freeDevice();
        return( false );
    }

    return( true );
}

bool MxGamePadDX::enumDevices()
{
    if( _pDI == 0 )
        return( false );

    _devList.clear();

    HRESULT rc = _pDI->EnumDevices( DI8DEVCLASS_GAMECTRL, enumDevicesCallback, this, DIEDFL_ATTACHEDONLY );
    if( rc != DI_OK )
        osg::notify( osg::WARN ) << "MxGamePadDX: enumDevices failed." << std::endl;

    return( rc == DI_OK );
}

void MxGamePadDX::freeDevice()
{
    if( _pDIDevice )
    {
        _pDIDevice->Unacquire();
        _pDIDevice->Release();
        _pDIDevice = 0;
    }
}

void MxGamePadDX::freeDirectInput()
{
    freeDevice();
    if( _pDI )
    {
        _pDI->Release();
        _pDI = 0;
    }
}

void MxGamePadDX::destroyWindow()
{
    if( _hDIWindow )
    {
        ::DestroyWindow( _hDIWindow );
        _hDIWindow = 0;
    }
}



BOOL CALLBACK EnumAxisCallback( const DIDEVICEOBJECTINSTANCE* pDIDOI, void* pContext )
{
    DIPROPRANGE pr;
    pr.diph.dwSize = sizeof( DIPROPRANGE );
    pr.diph.dwHeaderSize = sizeof( DIPROPHEADER );
    pr.diph.dwHow = DIPH_BYID;
    pr.diph.dwObj = pDIDOI->dwType;           // the axis being enumerated
    // set min & max range values. this is the whole purpose of enumerating the axis.
    pr.lMin = (LONG)( MIN_AXIS_VALUE );
    pr.lMax = (LONG)( MAX_AXIS_VALUE );
    ( ( LPDIRECTINPUTDEVICE8 )pContext )->SetProperty( DIPROP_RANGE, &pr.diph );
    return( DIENUM_CONTINUE );                   // let's do 'em all, thank you
}

BOOL CALLBACK enumDevicesCallback( const DIDEVICEINSTANCE* pdidInstance, void* pUser )
{
    ( ( MxGamePadDX* )pUser )->_devList.push_back( *pdidInstance );
    return( DIENUM_CONTINUE );                   // next please
}

LRESULT CALLBACK WinProc( HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    return( DefWindowProc( hwnd, uMsg, wParam, lParam ) );
}

inline float normalizeAxisValue( LONG value )
{
    return( ( ( value - MIN_AXIS_VALUE ) / HALF_AXIS_RANGE ) - 1.f );
}


// osgwMx
}
