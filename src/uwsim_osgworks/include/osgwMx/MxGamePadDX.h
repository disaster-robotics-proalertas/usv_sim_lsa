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

#ifndef __OSGWMX_MX_GAME_PAD_DX_H__
#define __OSGWMX_MX_GAME_PAD_DX_H__ 1


#include <osgwMx/Export.h>
#include <osgwMx/MxGamePad.h>
#include <osg/Object>

#include <dinput.h>

#include <list>


namespace osgwMx {


/** \class MxGamePadDX MxGamePadDX.h <osgwMx/MxGamePadDX.h>
\brief A DirectInput implementation of the MxGamePad class.
*/
class OSGWMX_EXPORT MxGamePadDX : public MxGamePad
{
public:
    MxGamePadDX();
    MxGamePadDX( const MxGamePadDX& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgwMx,MxGamePadDX);

    /** \brief Poll the gamepad and send device state to the base class.

    Calling application should invoke this function once per frame.

    If you are implementing another MxGamePad-based class, and it is
    non-polling, you'll need to code up some kind of function to handle
    your events. Use poll() as a model for how to call into the base
    class in response to the events you receive.
    */
    bool poll( const double elapsedSeconds );

protected:
    virtual ~MxGamePadDX();


    //
    // DX Support
    //

    // I feel so sorry for anyone who has to code to the Windows
    // API on a daily basis...

    static bool s_winClassRegistered;       // true if the window class has been registered

    friend static BOOL CALLBACK enumDevicesCallback(const DIDEVICEINSTANCE *pdidInstance, void *pContext);

    HWND _hDIWindow;                        // handle to the top-level window that DirectInput needs.
    LPDIRECTINPUT8 _pDI;                    // a pointer to the DirectInput v8 COM interface.
    LPDIRECTINPUTDEVICE8 _pDIDevice;        // pointer to the current device being used by this.
    DIDEVCAPS _devCaps;

    typedef std::list< DIDEVICEINSTANCE > DeviceList;
    DeviceList _devList;   // list of gaming devices on the system.

    HWND createInvisiWindow();
    bool openDirectInput();
    bool selectFirstDevice();
    bool enumDevices();
    bool selectDevice( const DIDEVICEINSTANCE& device );
    void freeDevice();
    void freeDirectInput();
    void destroyWindow();

    void processButtons( const DIJOYSTATE2& devState, const double deltaSeconds );
    void processSticks( const DIJOYSTATE2& devState, const double deltaSeconds );
    void processDPad( const DIJOYSTATE2& devState );
};


// osgwMx
}


// __OSGWMX_MX_GAME_PAD_DX_H__
#endif
